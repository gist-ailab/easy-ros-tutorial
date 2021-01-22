import torch
import torchvision.models as models
import torchvision.transforms as transforms

from easy_tcp_python2_3 import socket_utils as su
from imagenet_stubs.imagenet_2012_labels import label_to_name


if __name__ == "__main__" :

    sock = su.initialize_client('localhost', 7777)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = models.detection.maskrcnn_resnet50_fpn(pretrained=True).to(device)
    model.eval()
    print("Using mask R-CNN with ResNet-50 backbone", device)
    transforms_composed = transforms.Compose([
            transforms.ToTensor()])

    while True:
        
        recv_image = su.recvall_image(sock)
        x = transforms_composed(recv_image).unsqueeze(0)
        outputs = model(x.to(device))[0]

        probs = outputs['scores'].cpu().detach().numpy()
        boxes = outputs["boxes"].cpu().detach().numpy()
        labels = outputs["labels"].cpu().detach().numpy()
        masks = outputs["masks"].cpu().detach().numpy()
        
        keep = probs > 0.5
        probs = probs[keep]
        labels = labels[keep]
        boxes = boxes[keep]
        masks = masks[keep]

        su.sendall_pickle(sock, probs)
        su.sendall_pickle(sock, labels)
        su.sendall_pickle(sock, boxes)
        su.sendall_pickle(sock, masks)




