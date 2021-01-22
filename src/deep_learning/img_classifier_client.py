import torch
import torchvision.models as models
import torchvision.transforms as transforms

from easy_tcp_python2_3 import socket_utils as su
from imagenet_stubs.imagenet_2012_labels import label_to_name


if __name__ == "__main__" :

    sock = su.initialize_client('localhost', 7777)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = models.resnet50(pretrained=True).to(device)
    model.eval()
    print("Using ResNet-50", device)
    transforms_composed = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225]),
            ])

    while True:
        
        recv_image = su.recvall_image(sock)
        x = transforms_composed(recv_image).unsqueeze(0)
        outputs = model(x.to(device))
        _, preds = torch.max(outputs, 1)
        class_name = label_to_name(preds[0].item())
        su.sendall_pickle(sock, class_name)



