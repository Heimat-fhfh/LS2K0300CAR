// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

#include "inference.h"

Inference::Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::string &classesTxtFile, const bool &runWithCuda)
{
    modelPath = onnxModelPath;
    modelShape = modelInputShape;
    classesPath = classesTxtFile;
    cudaEnabled = runWithCuda;

    loadOnnxNetwork();
    // loadClassesFromFile(); The classes are hard-coded for this example
}

std::vector<Detection> Inference::runInference(const cv::Mat &input)
{
    cv::Mat modelInput = input;

    // IMPORTANT: initialize to safe defaults (prevents UB when letterbox is not used)
    int pad_x = 0, pad_y = 0;
    float scale = 1.0f;

    // Optional letterbox for square models
    if (letterBoxForSquare && modelShape.width == modelShape.height)
        modelInput = formatToSquare(modelInput, &pad_x, &pad_y, &scale);

    // Preprocess
    cv::Mat blob;
    cv::dnn::blobFromImage(modelInput, blob, 1.0 / 255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);

    // Forward
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    if (outputs.empty())
    {
        std::cerr << "Error: net.forward() returned empty outputs." << std::endl;
        return {};
    }

    cv::Mat out0 = outputs[0];

    // ------------------------------------------------------------
    // 1) CLASSIFICATION BRANCH (YOLOv8-cls)
    // Typical output: (1, C) => OpenCV Mat dims==2 and size = [1, C]
    // Your export log showed output shape(s) (1, 3).
    // ------------------------------------------------------------
    if (out0.dims == 2 && out0.size[0] == 1)
    {
        // out0 is 1 x C (may be CV_32F)
        // Convert to float if needed
        cv::Mat scores;
        if (out0.type() != CV_32F)
            out0.convertTo(scores, CV_32F);
        else
            scores = out0;

        // Flatten to 1 x C
        scores = scores.reshape(1, 1);

        const int numClasses = scores.cols;
        if (numClasses <= 0)
        {
            std::cerr << "Error: classification output has invalid class count." << std::endl;
            return {};
        }

        // Softmax (safe even if already probs; if already probs it will slightly change them but keeps argmax usually)
        // If you want "no softmax", you can remove this block, but logits are common.
        double maxVal = 0.0;
        cv::minMaxLoc(scores, nullptr, &maxVal);

        cv::Mat expScores;
        cv::exp(scores - maxVal, expScores);

        double sumExp = cv::sum(expScores)[0];
        cv::Mat prob = expScores / (sumExp > 0.0 ? sumExp : 1.0);

        cv::Point classIdPoint;
        double confidence = 0.0;
        cv::minMaxLoc(prob, nullptr, &confidence, nullptr, &classIdPoint);

        int class_id = classIdPoint.x;

        Detection result;
        result.class_id = class_id;
        result.confidence = static_cast<float>(confidence);

        // Random-ish color, consistent with your prior style
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(100, 255);
        result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

        if (class_id >= 0 && class_id < (int)classes.size())
            result.className = classes[class_id];
        else
            result.className = "unknown";

        // Keep main.cpp compatible: give a "box" (full image)
        // so cv::rectangle and text placement don't crash.
        result.box = cv::Rect(0, 0, input.cols, input.rows);

        return { result };
    }

    // Some models might output (C) as 1-D. Handle that too.
    if (out0.dims == 1)
    {
        cv::Mat scores1d = out0;
        if (scores1d.type() != CV_32F)
            scores1d.convertTo(scores1d, CV_32F);

        scores1d = scores1d.reshape(1, 1); // 1 x C

        double maxVal = 0.0;
        cv::minMaxLoc(scores1d, nullptr, &maxVal);

        cv::Mat expScores;
        cv::exp(scores1d - maxVal, expScores);

        double sumExp = cv::sum(expScores)[0];
        cv::Mat prob = expScores / (sumExp > 0.0 ? sumExp : 1.0);

        cv::Point classIdPoint;
        double confidence = 0.0;
        cv::minMaxLoc(prob, nullptr, &confidence, nullptr, &classIdPoint);

        int class_id = classIdPoint.x;

        Detection result;
        result.class_id = class_id;
        result.confidence = static_cast<float>(confidence);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(100, 255);
        result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

        if (class_id >= 0 && class_id < (int)classes.size())
            result.className = classes[class_id];
        else
            result.className = "unknown";

        result.box = cv::Rect(0, 0, input.cols, input.rows);
        return { result };
    }

    // ------------------------------------------------------------
    // 2) DETECTION BRANCH (your original logic, preserved)
    // Requires output to be 3-D so that size[2] exists.
    // ------------------------------------------------------------
    if (out0.dims < 3)
    {
        std::cerr << "Error: unexpected output dims=" << out0.dims
                  << ". This isn't a supported detect output, and it wasn't recognized as classification."
                  << std::endl;
        return {};
    }

    int rows = out0.size[1];
    int dimensions = out0.size[2];

    bool yolov8 = false;
    // yolov5 output: (batch, 25200, 85)   => dims=3, size[1]=25200, size[2]=85
    // yolov8 output: (batch, 84,   8400)  => dims=3, size[1]=84,    size[2]=8400
    if (dimensions > rows) // yolov8 heuristic
    {
        yolov8 = true;
        rows = out0.size[2];
        dimensions = out0.size[1];

        outputs[0] = outputs[0].reshape(1, dimensions);
        cv::transpose(outputs[0], outputs[0]);
    }

    float *data = (float *)outputs[0].data;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i)
    {
        if (yolov8)
        {
            float *classes_scores = data + 4;

            cv::Mat scores(1, (int)classes.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            if (maxClassScore > modelScoreThreshold)
            {
                confidences.push_back((float)maxClassScore);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5f * w - pad_x) / scale);
                int top  = int((y - 0.5f * h - pad_y) / scale);

                int width  = int(w / scale);
                int height = int(h / scale);

                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        else // yolov5
        {
            float confidence = data[4];

            if (confidence >= modelConfidenceThreshold)
            {
                float *classes_scores = data + 5;

                cv::Mat scores(1, (int)classes.size(), CV_32FC1, classes_scores);
                cv::Point class_id;
                double max_class_score;

                minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

                if (max_class_score > modelScoreThreshold)
                {
                    confidences.push_back(confidence);
                    class_ids.push_back(class_id.x);

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    int left = int((x - 0.5f * w - pad_x) / scale);
                    int top  = int((y - 0.5f * h - pad_y) / scale);

                    int width  = int(w / scale);
                    int height = int(h / scale);

                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
        }

        data += dimensions;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    std::vector<Detection> detections{};
    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(100, 255);
        result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

        result.className = (result.class_id >= 0 && result.class_id < (int)classes.size())
                            ? classes[result.class_id]
                            : "unknown";

        result.box = boxes[idx];

        detections.push_back(result);
    }

    return detections;
}


void Inference::loadClassesFromFile()
{
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open())
    {
        std::string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}

void Inference::loadOnnxNetwork()
{
    net = cv::dnn::readNetFromONNX(modelPath);
    if (cudaEnabled)
    {
        std::cout << "\nRunning on CUDA" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else
    {
        std::cout << "\nRunning on CPU" << std::endl;
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
}

cv::Mat Inference::formatToSquare(const cv::Mat &source, int *pad_x, int *pad_y, float *scale)
{
    int col = source.cols;
    int row = source.rows;
    int m_inputWidth = modelShape.width;
    int m_inputHeight = modelShape.height;

    *scale = std::min(m_inputWidth / (float)col, m_inputHeight / (float)row);
    int resized_w = col * *scale;
    int resized_h = row * *scale;
    *pad_x = (m_inputWidth - resized_w) / 2;
    *pad_y = (m_inputHeight - resized_h) / 2;

    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resized_w, resized_h));
    cv::Mat result = cv::Mat::zeros(m_inputHeight, m_inputWidth, source.type());
    resized.copyTo(result(cv::Rect(*pad_x, *pad_y, resized_w, resized_h)));
    resized.release();
    return result;
}
