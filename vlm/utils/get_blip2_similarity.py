"""Original ApexNav scene prompt with the independent BLIP2 ITC backend."""
from vlm.similarity.blip2 import BLIP2Client

blip2_client = BLIP2Client()


def get_blip2_similarity(rgb_image, label, room):
    if room != 'everywhere':
        text = f'Seems like there is a {room} or a {label} ahead?'
    else:
        text = f'Seems like there is a {label} ahead?'
    return blip2_client.cosine(rgb_image, text)
