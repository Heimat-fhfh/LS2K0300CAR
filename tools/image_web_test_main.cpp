#define MAKE_MAIN_IMPL

#include <filesystem>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "/home/fhfh/Work/LS2K0300CAR/third_party/cpp-httplib-master/httplib.h"
#include "common/common_program.h"
#include "vision/camera_calibration.h"
#include "common/json.hpp"
#include "vision/image_my_zf.h"

void ProcessTrackTaskPerFrame(Img_Store *Img_Store_p, Data_Path *Data_Path_p, Function_EN *Function_EN_p,ImgProcess *imgProcess_p,Judge *judge_p);
void ApplyDifferentialControl(Img_Store *Img_Store_p, Data_Path *Data_Path_p, Function_EN *Function_EN_p, Judge *judge_p);

Function_EN Function_EN_s;
JSON_PIDConfigData JSON_PIDConfigData_s;
Data_Path Data_Path_s;
SYNC Sync;
bool g_runtime_config_ok = false;
namespace fs = std::filesystem;
using json = nlohmann::json;

bool g_calibration_enabled = false;
CameraCalibrationCorrector g_calibration_corrector;

namespace
{

    struct AppConfig
    {
        std::string dataset_dir = "/home/fhfh/Work/LS2K0300CAR/docs/img/20260602_113553";
        std::string config_file = "config/config_0.json";
        std::string frontend_dir;
        std::string host = "0.0.0.0";
        int port = 3100;
    };

    struct FrameResult
    {
        std::vector<unsigned char> original_jpg;
        std::vector<unsigned char> gray_jpg;
        std::vector<unsigned char> otsu_jpg;
        std::vector<unsigned char> track_jpg;
        std::vector<unsigned char> all_jpg;

        int track_kind = 0;
        int my_zf_road_type = 0;
        int my_zf_det_true = 0;
        int my_zf_off_line = 0;
        int my_zf_ring_flag = 0;
        int my_zf_rings = 0;
        int my_zf_ring_size = 0;
        int my_zf_left_line = 0;
        int my_zf_right_line = 0;
        int my_zf_white_line = 0;
        int steer_error_px = 0;
        double target_base_speed = 0.0;
    };

    bool ends_with_jpg(const std::string &name)
    {
        if (name.size() < 4)
        {
            return false;
        }
        const std::string suffix = name.substr(name.size() - 4);
        return suffix == ".jpg" || suffix == ".JPG";
    }

    int parse_stem_index(const fs::path &p)
    {
        try
        {
            return std::stoi(p.stem().string());
        }
        catch (...)
        {
            return -1;
        }
    }

    std::string resolve_frontend_dir(const std::string &cli_path)
    {
        if (!cli_path.empty() && fs::exists(cli_path) && fs::is_directory(cli_path))
        {
            return cli_path;
        }

        const std::vector<std::string> candidates = {
            "web/image_web_test",
            "../web/image_web_test",
            "../../web/image_web_test"};

        for (const auto &p : candidates)
        {
            if (fs::exists(p) && fs::is_directory(p))
            {
                return p;
            }
        }
        return "";
    }

    void fill_runtime_config_from_json(const json &cfg,
                                       Function_EN *function_en,
                                       Data_Path *data_path)
    {
        JSON_FunctionConfigData function_cfg{};
        JSON_TrackConfigData track_cfg{};

        function_cfg.Uart_EN = cfg.at("UART_EN");
        function_cfg.ImgCompress_EN = cfg.at("IMG_COMPRESS_EN");
        function_cfg.Camera_EN = CameraKind(cfg.at("CAMERA_EN"));
        function_cfg.ImageSave_EN = false;
        function_cfg.VideoShow_EN = false;
        function_cfg.DataPrint_EN = false;
        function_cfg.AcrossIdentify_EN = cfg.at("ACROSS_IDENTIFY_EN");
        function_cfg.CircleIdentify_EN = cfg.at("CIRCLE_IDENTIFY_EN");

        track_cfg.Track_width = cfg.at("Track_width");

        track_cfg.Forward = cfg.at("FORWARD");
        track_cfg.Default_Forward = cfg.at("FORWARD");
        track_cfg.ForwardHeightCompensationPxPerRow = cfg.at("FORWARD_HEIGHT_COMPENSATION_PX_PER_ROW");
        track_cfg.Path_Search_Start = cfg.at("PATH_SEARCH_START");
        track_cfg.Path_Search_End = cfg.at("PATH_SEARCH_END");
        track_cfg.Side_Search_Start = cfg.at("SIDE_SEARCH_START");
        track_cfg.Side_Search_End = cfg.at("SIDE_SEARCH_END");

        track_cfg.InflectionPointVectorDistance = cfg.at("POINT_DISTANCE");
        track_cfg.BendPointVectorDistance = cfg.at("POINT_DISTANCE");
        track_cfg.BendPointNum[0] = cfg.at("LITTLE_ANGLE_BEND_POINT_NUM");
        track_cfg.BendPointNum[1] = cfg.at("BIG_ANGLE_BEND_POINT_NUM");
        track_cfg.InflectionPointIdentifyAngle[0] = cfg.at("MIN_INFLECTION_POINT_ANGLE");
        track_cfg.InflectionPointIdentifyAngle[1] = cfg.at("MAX_INFLECTION_POINT_ANGLE");
        track_cfg.BendPointIdentifyAngle[0] = cfg.at("MIN_BEND_POINT_ANGLE");
        track_cfg.BendPointIdentifyAngle[1] = cfg.at("MAX_BEND_POINT_ANGLE");

        track_cfg.TrackWidth = cfg.at("TRACK_WIDTH");
        track_cfg.TrackKindCountThreshold = cfg.at("TRACK_KIND_COUNT_THRESHOLD");
        track_cfg.CircleOutWidth = cfg.at("CIRCLE_OUT_WIDTH");
        track_cfg.CommonMotorSpeed[0] = cfg.at("STRIGHT_TRACK_MOTOR_SPEED");
        track_cfg.CommonMotorSpeed[1] = cfg.at("LITTLE_ANGLE_BEND_TRACK_MOTOR_SPEED");
        track_cfg.CommonMotorSpeed[2] = cfg.at("BIG_ANGLE_BEND_TRACK_MOTOR_SPEED");
        track_cfg.CommonMotorSpeed[3] = cfg.at("ACROSS_TRACK_MOTOR_SPEED");
        track_cfg.CommonMotorSpeed[4] = cfg.at("CIRCLE_TRACK_MOTOR_SPEED_OUTSIDE");
        track_cfg.CommonMotorSpeed[5] = cfg.at("CIRCLE_TRACK_MOTOR_SPEED_INSIDE");
        track_cfg.BridgeZoneMotorSpeed = cfg.at("BRIDGE_ZONE_MOTOR_SPEED");
        track_cfg.CrosswalkZoneMotorSpeed = cfg.at("CROSSWALK_ZONE_MOTOR_SPEED_STOP_PREPARE");
        track_cfg.Circle_In_Prepare_Time = cfg.at("CIRCLE_IN_PREPARE_TIME");

        if (cfg.contains("TRANSITION_MIN_AREA"))
        {
            track_cfg.TransitionMinArea = cfg.at("TRANSITION_MIN_AREA");
        }

        if (cfg.contains("CIRCLE_JUMP_MAX")) {
            track_cfg.CircleJumpMax = cfg.at("CIRCLE_JUMP_MAX");
        }
        if (cfg.contains("CIRCLE_BORDER_QUIET_MAX")) {
            track_cfg.CircleBorderQuietMax = cfg.at("CIRCLE_BORDER_QUIET_MAX");
        }
        if (cfg.contains("CIRCLE_JUMP_EXPECTED")) {
            track_cfg.CircleJumpExpected = cfg.at("CIRCLE_JUMP_EXPECTED");
        }
        if (cfg.contains("CIRCLE_BORDER_ACTIVE_MIN")) {
            track_cfg.CircleBorderActiveMin = cfg.at("CIRCLE_BORDER_ACTIVE_MIN");
        }
        if (cfg.contains("CIRCLE_JUDGE_INFLECTION_MAX")) {
            track_cfg.CircleJudgeInflectionMax = cfg.at("CIRCLE_JUDGE_INFLECTION_MAX");
        }
        if (cfg.contains("CIRCLE_JUDGE_PARTIAL_SCORE")) {
            track_cfg.CircleJudgePartialScore = cfg.at("CIRCLE_JUDGE_PARTIAL_SCORE");
        }
        if (cfg.contains("CROSS_JUMP_PRIMARY")) {
            track_cfg.CrossJumpPrimary = cfg.at("CROSS_JUMP_PRIMARY");
        }
        if (cfg.contains("CROSS_JUMP_SECONDARY_MIN")) {
            track_cfg.CrossJumpSecondaryMin = cfg.at("CROSS_JUMP_SECONDARY_MIN");
        }
        if (cfg.contains("CROSS_JUMP_SECONDARY_MAX")) {
            track_cfg.CrossJumpSecondaryMax = cfg.at("CROSS_JUMP_SECONDARY_MAX");
        }
        if (cfg.contains("CROSS_BORDER_MIN")) {
            track_cfg.CrossBorderMin = cfg.at("CROSS_BORDER_MIN");
        }
        if (cfg.contains("ACROSS_BORDER_PREPARE_MAX")) {
            track_cfg.AcrossBorderPrepareMax = cfg.at("ACROSS_BORDER_PREPARE_MAX");
        }
        if (cfg.contains("ACROSS_BORDER_OUT_MIN")) {
            track_cfg.AcrossBorderOutMin = cfg.at("ACROSS_BORDER_OUT_MIN");
        }
        if (cfg.contains("ACROSS_BORDER_EXIT_MAX")) {
            track_cfg.AcrossBorderExitMax = cfg.at("ACROSS_BORDER_EXIT_MAX");
        }
        if (cfg.contains("ACROSS_MAX_FRAMES")) {
            track_cfg.AcrossMaxFrames = cfg.at("ACROSS_MAX_FRAMES");
        }
        if (cfg.contains("TRACK_JUDGE_FULL_SCORE")) {
            track_cfg.TrackJudgeFullScore = cfg.at("TRACK_JUDGE_FULL_SCORE");
        }
        if (cfg.contains("TRACK_JUDGE_PARTIAL_SCORE")) {
            track_cfg.TrackJudgePartialScore = cfg.at("TRACK_JUDGE_PARTIAL_SCORE");
        }
        if (cfg.contains("TRACK_JUDGE_CONFIRM_THRESHOLD")) {
            track_cfg.TrackJudgeConfirmThreshold = cfg.at("TRACK_JUDGE_CONFIRM_THRESHOLD");
        }

        function_en->JSON_FunctionConfigData_v.clear();
        data_path->JSON_TrackConfigData_v.clear();
        function_en->JSON_FunctionConfigData_v.push_back(function_cfg);
        data_path->JSON_TrackConfigData_v.push_back(track_cfg);
        function_en->Game_EN = true;
        function_en->Gyroscope_EN = false;
        data_path->Loop_Kind = CAMERA_CATCH_LOOP;
        function_en->Control_EN = false;

        data_path->Track_Kind = STRIGHT_TRACK;
        data_path->Circle_Track_Step = INIT_CIRCLE;
        data_path->Track_Kind = STRIGHT_TRACK;
    }

    std::vector<unsigned char> encode_jpeg(const cv::Mat &image)
    {
        if (image.empty())
        {
            return {};
        }
        std::vector<unsigned char> encoded;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
        cv::imencode(".jpg", image, encoded, params);
        return encoded;
    }

    class OfflineImageProcessor
    {
    public:
        bool init(const AppConfig &cfg, std::string *error)
        {
            cfg_ = cfg;
            frame_files_.clear();

            if (!fs::exists(cfg_.dataset_dir))
            {
                if (error)
                {
                    *error = "Dataset dir not found: " + cfg_.dataset_dir;
                }
                return false;
            }

            for (const auto &entry : fs::directory_iterator(cfg_.dataset_dir))
            {
                if (!entry.is_regular_file())
                {
                    continue;
                }
                const std::string filename = entry.path().filename().string();
                if (ends_with_jpg(filename))
                {
                    frame_files_.push_back(entry.path());
                }
            }

            std::sort(frame_files_.begin(), frame_files_.end(), [](const fs::path &a, const fs::path &b)
                      {
      const int ia = parse_stem_index(a);
      const int ib = parse_stem_index(b);
      if (ia >= 0 && ib >= 0) {
        return ia < ib;
      }
      return a.filename().string() < b.filename().string(); });

            if (frame_files_.empty())
            {
                if (error)
                {
                    *error = "No jpg found in: " + cfg_.dataset_dir;
                }
                return false;
            }

            std::ifstream ifs(cfg_.config_file);
            if (!ifs.is_open())
            {
                if (error)
                {
                    *error = "Cannot open config file: " + cfg_.config_file;
                }
                return false;
            }

            json cfg_json;
            try
            {
                ifs >> cfg_json;
                fill_runtime_config_from_json(cfg_json, &function_en_, &data_path_);
            }
            catch (const std::exception &e)
            {
                if (error)
                {
                    *error = std::string("Config parse failed: ") + e.what();
                }
                return false;
            }

            return true;
        }

        int frame_count() const
        {
            return static_cast<int>(frame_files_.size());
        }

        std::string frame_name(int idx) const
        {
            return frame_files_.at(static_cast<size_t>(idx)).filename().string();
        }

        bool get_frame_result(int idx, FrameResult *out, std::string *error)
        {
            if (out == nullptr)
            {
                if (error)
                {
                    *error = "output pointer is null";
                }
                return false;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            if (idx < 0 || idx >= frame_count())
            {
                if (error)
                {
                    *error = "idx out of range";
                }
                return false;
            }

            if (cache_idx_ == idx && cache_.has_value())
            {
                *out = *cache_;
                return true;
            }

            Img_Store img_store;
            img_store.ImgNum = idx;
            img_store.Img_Color = cv::imread(frame_files_[static_cast<size_t>(idx)].string(), cv::IMREAD_COLOR);
            if (img_store.Img_Color.empty())
            {
                if (error)
                {
                    *error = "Failed to read image: " + frame_files_[static_cast<size_t>(idx)].string();
                }
                return false;
            }

            // img_process_.ImgCompress(img_store.Img_Color, true);
            // img_process_.imgPreProc(&img_store, &data_path_, &function_en_);

            // imgSearch_l_r(&img_store, &data_path_);
            // judge_.TransitionScanDetect(&img_store, &data_path_, &function_en_);

            // judge_.Search_Data_Analysis(&img_store, &data_path_, &function_en_);
            // judge_.TrackKind_Judge(&img_store, &data_path_, &function_en_);
            
            ProcessTrackTaskPerFrame(&img_store, &data_path_, &function_en_,&img_process_ ,&judge_);

            // judge_.ServoDirAngle_Judge(&data_path_);
            // judge_.MotorSpeed_Judge(&img_store, &data_path_);
            // judge_.AngularVelocityTarget_Judge(&data_path_);
            ApplyDifferentialControl(&img_store, &data_path_, &function_en_, &judge_);
            img_process_.ImgShow(&img_store, &data_path_, &function_en_);

            FrameResult current;
            current.original_jpg = encode_jpeg(img_store.Img_Color);
            current.gray_jpg = encode_jpeg(img_store.Img_Gray);
            current.otsu_jpg = encode_jpeg(img_store.Img_OTSU);
            current.track_jpg = encode_jpeg(img_store.Img_Track);
            current.all_jpg = encode_jpeg(img_store.Img_All);

            current.track_kind = static_cast<int>(data_path_.Track_Kind);
            current.my_zf_road_type = static_cast<int>(ImageStatus.Road_type);
            current.my_zf_det_true = ImageStatus.Det_True;
            current.my_zf_off_line = ImageStatus.OFFLine;
            current.my_zf_ring_flag = ImageFlag.image_element_rings_flag;
            current.my_zf_rings = ImageFlag.image_element_rings;
            current.my_zf_ring_size = ImageFlag.ring_big_small;
            current.my_zf_left_line = ImageStatus.Left_Line;
            current.my_zf_right_line = ImageStatus.Right_Line;
            current.my_zf_white_line = ImageStatus.WhiteLine;
            current.steer_error_px = data_path_.SteerErrorPx;
            current.target_base_speed = data_path_.TargetBaseSpeedMps;

            cache_ = current;
            cache_idx_ = idx;
            *out = current;
            return true;
        }

    private:
        AppConfig cfg_;
        std::vector<fs::path> frame_files_;
        ImgProcess img_process_;
        Judge judge_;
        Function_EN function_en_;
        Data_Path data_path_;

        std::mutex mutex_;
        std::optional<FrameResult> cache_;
        int cache_idx_ = -1;
    };

    int parse_int_or_default(const std::string &text, int fallback)
    {
        try
        {
            return std::stoi(text);
        }
        catch (...)
        {
            return fallback;
        }
    }

    AppConfig parse_args(int argc, char **argv)
    {
        AppConfig cfg;
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--dataset" && i + 1 < argc)
            {
                cfg.dataset_dir = argv[++i];
            }
            else if (arg == "--config" && i + 1 < argc)
            {
                cfg.config_file = argv[++i];
            }
            else if (arg == "--frontend-dir" && i + 1 < argc)
            {
                cfg.frontend_dir = argv[++i];
            }
            else if (arg == "--host" && i + 1 < argc)
            {
                cfg.host = argv[++i];
            }
            else if (arg == "--port" && i + 1 < argc)
            {
                cfg.port = parse_int_or_default(argv[++i], 8080);
            }
        }
        return cfg;
    }

} // namespace

int main(int argc, char **argv)
{
    // 5. 配置文件同步
    Sync.ConfigData_SYNC(&Data_Path_s,&Function_EN_s,&JSON_PIDConfigData_s);
    g_runtime_config_ok = !(Function_EN_s.JSON_FunctionConfigData_v.empty() || Data_Path_s.JSON_TrackConfigData_v.empty());
    if (g_runtime_config_ok)
    {
        std::string calibrationError;
        const std::string calibrationJsonPath = "config/calibration.json";
        const std::string calibrationYamlPath = "config/calibration.yaml";

        if (g_calibration_enabled)
        {
            g_calibration_enabled = g_calibration_corrector.load(calibrationYamlPath, &calibrationError);
            if (!g_calibration_enabled)
            {
                std::cerr << "[Calibration] YAML 加载失败: " << calibrationError << std::endl;
            }
            else
            {
                std::cout << "[Calibration] 已加载 YAML 标定参数: " << calibrationYamlPath << std::endl;
            }
        }
        else
        {
            std::cout << "[Calibration] 图像标定使能关闭: " << calibrationJsonPath << std::endl;
        }
    }
    else
    {
        std::cerr << "[Config] 配置同步失败" << std::endl;
        std::cerr << "[Config] Function 配置数量: " << Function_EN_s.JSON_FunctionConfigData_v.size()
                  << ", Track 配置数量: " << Data_Path_s.JSON_TrackConfigData_v.size() << std::endl;
        if (Function_EN_s.JSON_FunctionConfigData_v.empty())
        {
            std::cerr << "[Config] JSON_FunctionConfigData_v 为空，请检查功能配置文件" << std::endl;
        }
        if (Data_Path_s.JSON_TrackConfigData_v.empty())
        {
            std::cerr << "[Config] JSON_TrackConfigData_v 为空，请检查赛道配置文件" << std::endl;
        }
        Function_EN_s.Game_EN = false;
    }
    
    const AppConfig cfg = parse_args(argc, argv);

    OfflineImageProcessor processor;
    std::string init_error;
    if (!processor.init(cfg, &init_error))
    {
        std::cerr << "[ImageWebTest] init failed: " << init_error << std::endl;
        return 1;
    }

    httplib::Server server;
    server.set_default_headers({
        {"Cache-Control", "no-store, no-cache, must-revalidate, max-age=0"},
        {"Pragma", "no-cache"},
        {"Expires", "0"}
    });
    const std::string frontend_dir = resolve_frontend_dir(cfg.frontend_dir);
    if (!frontend_dir.empty())
    {
        server.set_mount_point("/ui", frontend_dir);
        server.Get("/", [&](const httplib::Request &, httplib::Response &res)
                   { res.set_redirect("/ui/index.html"); });
    }
    else
    {
        server.Get("/", [&](const httplib::Request &, httplib::Response &res)
                   { res.set_content("Frontend not found. Use --frontend-dir to specify static files path.",
                                     "text/plain; charset=UTF-8"); });
    }

    server.Get("/api/meta", [&](const httplib::Request &, httplib::Response &res)
               {
    json meta;
    meta["frame_count"] = processor.frame_count();
    meta["dataset"] = cfg.dataset_dir;
    res.set_content(meta.dump(), "application/json; charset=UTF-8"); });

    server.Get("/api/frame", [&](const httplib::Request &req, httplib::Response &res)
               {
    const int idx = parse_int_or_default(req.get_param_value("idx"), 0);
    FrameResult frame;
    std::string error;
    if (!processor.get_frame_result(idx, &frame, &error)) {
      res.status = 400;
      res.set_content(std::string("{\"error\":\"") + error + "\"}", "application/json");
      return;
    }

    json stat;
    stat["file"] = processor.frame_name(idx);
    stat["track_kind"] = frame.track_kind;
    stat["my_zf_road_type"] = frame.my_zf_road_type;
    stat["my_zf_det_true"] = frame.my_zf_det_true;
    stat["my_zf_off_line"] = frame.my_zf_off_line;
    stat["my_zf_ring_flag"] = frame.my_zf_ring_flag;
    stat["my_zf_rings"] = frame.my_zf_rings;
    stat["my_zf_ring_size"] = frame.my_zf_ring_size;
    stat["my_zf_white_line"] = frame.my_zf_white_line;
    stat["steer_error_px"] = frame.steer_error_px;
    stat["target_base_speed"] = frame.target_base_speed;
    res.set_content(stat.dump(), "application/json; charset=UTF-8"); });

    server.Get("/api/image", [&](const httplib::Request &req, httplib::Response &res)
               {
    const int idx = parse_int_or_default(req.get_param_value("idx"), 0);
    const std::string type = req.has_param("type") ? req.get_param_value("type") : "orig";

    FrameResult frame;
    std::string error;
    if (!processor.get_frame_result(idx, &frame, &error)) {
      res.status = 400;
      res.set_content(error, "text/plain; charset=UTF-8");
      return;
    }

    const std::vector<unsigned char>* payload = &frame.original_jpg;
    if (type == "gray") {
        payload = &frame.gray_jpg;
    } else if (type == "otsu") {
        payload = &frame.otsu_jpg;
    } else if (type == "track") {
        payload = &frame.track_jpg;
    } else if (type == "all") {
        payload = &frame.all_jpg;
    }

    if (payload->empty()) {
      res.status = 500;
      res.set_content("empty image payload", "text/plain; charset=UTF-8");
      return;
    }
    res.set_content(reinterpret_cast<const char*>(payload->data()), payload->size(), "image/jpeg"); });

    std::cout << "[ImageWebTest] dataset=" << cfg.dataset_dir << std::endl;
    std::cout << "[ImageWebTest] config=" << cfg.config_file << std::endl;
    if (!frontend_dir.empty())
    {
        std::cout << "[ImageWebTest] frontend=" << frontend_dir << std::endl;
        std::cout << "[ImageWebTest] open http://127.0.0.1:" << cfg.port << "/ui/index.html" << std::endl;
    }
    else
    {
        std::cout << "[ImageWebTest] frontend=NOT_FOUND" << std::endl;
        std::cout << "[ImageWebTest] open http://127.0.0.1:" << cfg.port << " (API only)" << std::endl;
    }
    server.listen(cfg.host, cfg.port);
    return 0;
}
