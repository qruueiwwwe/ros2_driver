#!/bin/bash

# 设置变量
IMAGE_NAME="ros2-driver"
IMAGE_TAG="latest"
K3S_REGISTRY="localhost:5000"  # 如果使用本地 K3s 注册表

# 构建 Docker 镜像
echo "构建 Docker 镜像..."
docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .

# 如果需要推送到 K3s 注册表
# echo "推送镜像到 K3s 注册表..."
# docker tag ${IMAGE_NAME}:${IMAGE_TAG} ${K3S_REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}
# docker push ${K3S_REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}

# 部署到 K3s 集群
echo "部署到 K3s 集群..."
kubectl apply -f k8s-deployment.yaml

# 等待部署完成
echo "等待部署完成..."
kubectl rollout status deployment/ros2-driver

# 获取服务端口
NODE_PORT=$(kubectl get svc ros2-driver-service -o jsonpath='{.spec.ports[0].nodePort}')
NODE_IP=$(kubectl get nodes -o jsonpath='{.items[0].status.addresses[?(@.type=="InternalIP")].address}')

echo "部署完成！"
echo "您可以通过以下地址访问 ROS2 驱动服务："
echo "http://${NODE_IP}:${NODE_PORT}"
echo ""
echo "使用以下命令查看日志："
echo "kubectl logs -l app=ros2-driver" 