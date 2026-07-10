# Lecture 14: Docker 入门

## 学习路径

1. 安装 Docker CE + 配置用户组 + 镜像加速器
2. 学习 Docker 核心概念：镜像/容器/Dockerfile/仓库
3. 编写 Dockerfile（参考 26_SP install.md 依赖项）
4. 在容器中编译运行 26_SP demo

## Dockerfile 骨架

```dockerfile
FROM ubuntu:22.04
RUN apt update && apt install -y \
    build-essential cmake git \
    libopencv-dev libeigen3-dev \
    libyaml-cpp-dev libspdlog-dev libfmt-dev
# ... 更多依赖参考 26_SP install.md
WORKDIR /workspace
CMD ["/bin/bash"]
```

请根据 26_SP install.md 完成完整的 Dockerfile。
