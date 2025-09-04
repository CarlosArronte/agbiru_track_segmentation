import numpy as np

def segment_track_by_curvature(waypoints, smooth_threshold=0.0003, sharp_threshold=0.01, fusion_threshold=3):
    waypoints = np.array(waypoints)
    N = waypoints.shape[0]

    # === Cálculo de la curvatura ===
    curvatures = np.zeros(N - 2)
    for i in range(1, N - 1):
        A, B, C = waypoints[i - 1], waypoints[i], waypoints[i + 1]
        a = np.linalg.norm(B - C)
        b = np.linalg.norm(A - C)
        c = np.linalg.norm(A - B)
        s = (a + b + c) / 2
        area = np.sqrt(max(s * (s - a) * (s - b) * (s - c), 0))
        if area == 0 or a * b * c == 0:
            curvatures[i - 1] = 0
        else:
            curvatures[i - 1] = (4 * area) / (a * b * c)

    # === Clasificación por curvatura ===
    classes = []
    for k in curvatures:
        if k < smooth_threshold:
            classes.append("reta")
        elif k < sharp_threshold:
            classes.append("curva_suave")
        else:
            classes.append("curva_fechada")

    # === Agrupamiento inicial ===
    segments = []
    current_class = classes[0]
    idx_start = 0
    for i in range(1, len(classes)):
        if classes[i] != current_class:
            segments.append({
                'tipo': current_class,
                'indices': list(range(idx_start, i + 1))
            })
            idx_start = i
            current_class = classes[i]
    segments.append({
        'tipo': current_class,
        'indices': list(range(idx_start, len(classes) + 2))
    })

    # === Fusión de segmentos similares próximos ===
    merged_segments = []
    i = 0
    while i < len(segments):
        current = segments[i]
        j = i + 1
        while j < len(segments):
            next_seg = segments[j]
            if (next_seg['indices'][0] - current['indices'][-1] <= fusion_threshold) and \
                    next_seg['tipo'] == current['tipo']:
                current['indices'] = list(range(current['indices'][0], next_seg['indices'][-1] + 1))
                j += 1
            else:
                break
        merged_segments.append(current)
        i = j
    segments = merged_segments

    # === Homogeneización de tipo dominante ===
    for seg in segments:
        seg['tipo'] = seg['tipo']  # (ya es único por diseño)

    # === Ajustes por peligrosidad y homogeneización intermedia ===
    tipo_peligrosidad = {'reta': 1, 'curva_suave': 2, 'curva_fechada': 3}
    for k in range(1, len(segments) - 1):
        prev_seg = segments[k - 1]
        curr_seg = segments[k]
        next_seg = segments[k + 1]

        len_prev = len(prev_seg['indices'])
        len_curr = len(curr_seg['indices'])
        len_next = len(next_seg['indices'])

        if tipo_peligrosidad[prev_seg['tipo']] > tipo_peligrosidad[curr_seg['tipo']] and len_prev == len_curr:
            curr_seg['tipo'] = prev_seg['tipo']
        elif len_prev > len_curr and prev_seg['tipo'] == next_seg['tipo']:
            curr_seg['tipo'] = prev_seg['tipo']

    # === Fusión de segmentos pequeños entre homogéneos grandes ===
    for k in range(1, len(segments) - 1):
        prev_seg = segments[k - 1]
        curr_seg = segments[k]
        next_seg = segments[k + 1]
        if prev_seg['tipo'] == next_seg['tipo'] and curr_seg['tipo'] != prev_seg['tipo']:
            len_prev = len(prev_seg['indices'])
            len_curr = len(curr_seg['indices'])
            len_next = len(next_seg['indices'])
            menor_grande = min(len_prev, len_next)
            if menor_grande >= len_curr:
                curr_seg['tipo'] = prev_seg['tipo']

    # === Asociar waypoints ===
    for seg in segments:
        seg['waypoints'] = waypoints[seg['indices']]

    return segments
