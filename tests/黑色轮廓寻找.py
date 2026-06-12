import argparse
from pathlib import Path

import cv2
import numpy as np

def find_black_holes_contour_hierarchy(binary_img, min_area=60):
    """
    利用轮廓层级关系检测被白色包围的黑色区域
    
    Args:
        binary_img: 二值图像（白色=255, 黑色=0）
        min_area: minimum area to keep
    Returns:
        holes: 检测到的孔洞轮廓列表
        result_img: 标记了孔洞的结果图像
    """
    # 确保图像是二值的
    if len(binary_img.shape) > 2:
        binary_img = cv2.cvtColor(binary_img, cv2.COLOR_BGR2GRAY)
    
    _, binary_img = cv2.threshold(binary_img, 127, 255, cv2.THRESH_BINARY)

    # Find contours with full hierarchy
    contour_info = cv2.findContours(
        binary_img,
        cv2.RETR_TREE,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if len(contour_info) == 3:
        _, contours, hierarchy = contour_info
    else:
        contours, hierarchy = contour_info
    
    holes = []
    result_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
    
    if hierarchy is None:
        return holes, result_img
    
    hierarchy = hierarchy[0]  # 获取层级信息
    
    for i, (cnt, hier) in enumerate(zip(contours, hierarchy)):
        # hierarchy: [Next, Previous, First_Child, Parent]
        parent_idx = hier[3]
        
        # 有父轮廓的轮廓，即内部轮廓（孔洞）
        if parent_idx != -1:
            area = cv2.contourArea(cnt)
            # 可以根据面积过滤太小的噪点
            if area > min_area:
                holes.append(cnt)
                # 在结果图上绘制孔洞（红色）
                cv2.drawContours(result_img, [cnt], -1, (0, 0, 255), 2)
                # 填充孔洞以便可视化
                cv2.drawContours(result_img, [cnt], -1, (0, 0, 255), cv2.FILLED)
    
    return holes, result_img


def _ensure_odd(value):
    value = int(value)
    return value if value % 2 == 1 else value + 1


def binarize_gray_otsu(image, blur_ksize=5, invert=False):
    if blur_ksize > 1:
        blur_ksize = _ensure_odd(blur_ksize)
        image = cv2.GaussianBlur(image, (blur_ksize, blur_ksize), 0)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh_type = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    _, binary = cv2.threshold(gray, 0, 255, thresh_type | cv2.THRESH_OTSU)
    return binary


def detect_black_holes(
    image,
    min_area=60,
    blur_ksize=5,
    invert=False,
):
    binary = binarize_gray_otsu(image, blur_ksize=blur_ksize, invert=invert)
    holes, _ = find_black_holes_contour_hierarchy(binary, min_area=min_area)

    overlay = image.copy()
    if holes:
        cv2.drawContours(overlay, holes, -1, (0, 0, 255), 2)
    return overlay, binary, holes


def process_directory(
    input_dir,
    output_dir,
    pattern,
    min_area,
    blur_ksize,
    invert,
    save_mask,
    save_holes_mask,
    max_images,
    scale,
):
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    mask_dir = output_dir / "mask"
    holes_dir = output_dir / "holes_mask"
    if save_mask:
        mask_dir.mkdir(parents=True, exist_ok=True)
    if save_holes_mask:
        holes_dir.mkdir(parents=True, exist_ok=True)

    patterns = [p.strip() for p in pattern.split(",") if p.strip()]
    image_paths = []
    for pat in patterns:
        image_paths.extend(input_dir.glob(pat))
    image_paths = sorted(set(image_paths))

    if max_images is not None:
        image_paths = image_paths[: max_images]

    if not image_paths:
        print(f"No images found in {input_dir} with pattern {pattern}")
        return

    total_holes = 0
    processed = 0
    skipped = 0

    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            skipped += 1
            continue

        if scale != 1.0:
            image = cv2.resize(
                image,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_AREA,
            )

        overlay, white_mask, holes = detect_black_holes(
            image,
            min_area=min_area,
            blur_ksize=blur_ksize,
            invert=invert,
        )

        total_holes += len(holes)
        processed += 1

        out_path = output_dir / image_path.name
        cv2.imwrite(str(out_path), overlay)

        if save_mask:
            cv2.imwrite(str(mask_dir / image_path.name), white_mask)

        if save_holes_mask:
            holes_mask = np.zeros_like(white_mask)
            if holes:
                cv2.drawContours(holes_mask, holes, -1, 255, cv2.FILLED)
            cv2.imwrite(str(holes_dir / image_path.name), holes_mask)

    print(
        f"Processed {processed} images, total holes {total_holes}. "
        f"Skipped {skipped} unreadable images."
    )


def build_arg_parser():
    repo_root = Path(__file__).resolve().parents[1]
    default_input = repo_root / "document" / "img" / "20260409_141917"
    default_output = Path(__file__).resolve().parent / "output_black_holes_20260409_141917"

    parser = argparse.ArgumentParser(
        description="Detect black holes enclosed by white regions.",
    )
    parser.add_argument("--input-dir", default=str(default_input))
    parser.add_argument("--output-dir", default=str(default_output))
    parser.add_argument("--pattern", default="*.jpg")
    parser.add_argument("--min-area", type=float, default=1000)
    parser.add_argument("--blur-ksize", type=int, default=5)
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("--save-mask", action="store_true")
    parser.add_argument("--save-holes-mask", action="store_true")
    parser.add_argument("--max-images", type=int, default=None)
    parser.add_argument("--scale", type=float, default=1.0)
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    process_directory(
        input_dir=args.input_dir,
        output_dir=args.output_dir,
        pattern=args.pattern,
        min_area=args.min_area,
        blur_ksize=args.blur_ksize,
        invert=args.invert,
        save_mask=args.save_mask,
        save_holes_mask=args.save_holes_mask,
        max_images=args.max_images,
        scale=args.scale,
    )


if __name__ == "__main__":
    main()