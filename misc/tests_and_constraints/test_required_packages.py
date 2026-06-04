import sys
import subprocess
import shutil

# --- Formatting codes ---
GREEN = "\033[1;32m"
RED = "\033[1;31m"
RESET = "\033[0m"

all_checks_passed = True

def check(condition, message):
    """Helper to track pass/fail status"""
    global all_checks_passed
    if condition:
        print(f"[{GREEN}OK{RESET}] {message}")
    else:
        print(f"[{RED}FAIL{RESET}] {message}")
        all_checks_passed = False

print("-" * 50)
print("Startup Environment Check")
print("-" * 50)

# 1. Check nvidia-smi
nvidia_smi_path = shutil.which("nvidia-smi")
check(nvidia_smi_path is not None, "nvidia-smi available")

# 2. Check nvcc
try:
    nvcc_proc = subprocess.run(['nvcc', '--version'], capture_output=True, text=True)
    nvcc_installed = (nvcc_proc.returncode == 0)
except FileNotFoundError:
    nvcc_installed = False
check(nvcc_installed, "nvcc installed")

# # 3. Check Torch
# try:
#     import torch
#     current_torch = torch.__version__
#     target_torch = "2.5.0+cu124"
#     print(f"   > Torch version: {current_torch}")
#     check(current_torch == target_torch, f"Torch version matches {target_torch}")
    
#     # 4. Check CUDA Available (Only runs if torch exists)
#     cuda_avail = torch.cuda.is_available()
#     check(cuda_avail, "Torch CUDA available")

#     # 5. Check CUDA Memory Copy (Only runs if torch exists)
#     can_use_cuda = False
#     try:
#         if cuda_avail:
#             t = torch.rand(3, 3, device='cuda')
#             can_use_cuda = True
#             print(f"   > Tensor on device: {t.device}")
#     except Exception as e:
#         print(f"   > Error moving tensor to CUDA: {e}")
#     check(can_use_cuda, "Torch tensor copy to CUDA")

# except ImportError:
#     check(False, "Torch installed")
#     # Mark dependent checks as failed automatically
#     check(False, "Torch CUDA available (Skipped: Torch missing)")
#     check(False, "Torch tensor copy to CUDA (Skipped: Torch missing)")

# # 6. Check TorchVision
# try:
#     import torchvision
#     current_tv = torchvision.__version__
#     target_tv = "0.20.0+cu124"
#     print(f"   > TorchVision version: {current_tv}")
#     check(current_tv == target_tv, f"TorchVision version matches {target_tv}")
# except ImportError:
#     check(False, "TorchVision installed")

# # 7. Check ONNX Runtime
# try:
#     import onnxruntime as ort
#     current_ort = ort.__version__
#     target_ort = "1.20.0" 
#     providers = ort.get_available_providers()
#     print(f"   > ONNX Runtime version: {current_ort}")
#     print(f"   > ONNX Providers: {providers}")

#     ort_version_match = (current_ort == target_ort)
#     ort_cuda_provider = "CUDAExecutionProvider" in providers

#     check(ort_version_match, f"ONNX Runtime version matches {target_ort}")
#     check(ort_cuda_provider, "ONNX Runtime has CUDAExecutionProvider")
# except ImportError:
#     check(False, "ONNX Runtime installed")
#     check(False, "ONNX Runtime has CUDAExecutionProvider (Skipped: ONNX missing)")

# # 8. Check Numpy
# try:
#     import numpy as np
#     current_numpy = np.__version__
#     target_numpy = "1.23.0"
#     print(f"   > Numpy version: {current_numpy}")
#     check(current_numpy == target_numpy, f"Numpy version matches {target_numpy}")
# except ImportError:
#     check(False, "Numpy installed")

# 9. Check OpenCV
try:
    import cv2
    current_cv = cv2.__version__
    
    def is_version_ge(current, target):
        c_parts = [int(x) for x in current.split('.')]
        t_parts = [int(x) for x in target.split('.')]
        return c_parts >= t_parts

    target_cv_min = "4.5.0"
    print(f"   > OpenCV version: {current_cv}")
    check(is_version_ge(current_cv, target_cv_min), f"OpenCV >= {target_cv_min}")
except ImportError:
    check(False, "OpenCV installed")

print("-" * 50)

if all_checks_passed:
    print(f"""{GREEN}
      ___
     /  /
    /  /
___/  /
\\____/   All Systems Go!
{RESET}""")
else:
    print(f"{RED}❌ Some checks failed. Please review the errors above.{RESET}")
    print(f"For ros2 tag only numpy and opencv have to be [{GREEN}OK{RESET}].")