# Lecture 14 答案参考

参考 26_SP `install.md` 的完整依赖列表编写 Dockerfile。

关键依赖：
- OpenCV 4.10 (with CUDA)
- Eigen3, fmt, spdlog, yaml-cpp, nlohmann-json
- Ceres 2.2.0, OpenVINO 2024
- CUDA/TensorRT (容器中可选，需要 nvidia-docker)
