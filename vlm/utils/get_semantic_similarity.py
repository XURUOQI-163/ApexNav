"""Select independent inference clients without loading either model locally."""


def get_semantic_similarity(rgb_image, label, room, backend='clip'):
    if backend == 'clip':
        from vlm.utils.get_clip_similarity import get_clip_similarity
        return get_clip_similarity(rgb_image, label, room)
    if backend == 'blip2':
        from vlm.utils.get_blip2_similarity import get_blip2_similarity
        return get_blip2_similarity(rgb_image, label, room)
    raise ValueError(f'Unknown semantic backend: {backend}')
