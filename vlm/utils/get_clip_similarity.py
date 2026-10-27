"""Scene guidance using raw CLIP cosine similarity (not an ITM probability)."""
from vlm.similarity.clip import CLIPClient

clip_client = CLIPClient()


def get_clip_similarity(rgb_image, label, room):
    if room != 'everywhere':
        text = f'Seems like there is a {room} or a {label} ahead?'
    else:
        text = f'Seems like there is a {label} ahead?'
    return clip_client.cosine(rgb_image, text)
