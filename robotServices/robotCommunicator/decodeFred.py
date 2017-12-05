#!/usr/bin/env python3
'''This modules graphes and collects antenna data'''

import numpy as np
import scipy.cluster

PATTERN_END = "000000000000001"


def decode_frame(data_frame):
    '''This function decodes a frame'''

    # on tague une valeur pour chaque palier
    data_frame_orig = np.asarray(data_frame)
    k_means_orig = scipy.cluster.vq.kmeans(data_frame_orig[0:len(data_frame_orig)].astype(float), 2)[0]
    seuil_orig = np.average(k_means_orig)

    data_frame_np = data_frame_orig > seuil_orig
    data_frame_bumps = abs(np.diff(data_frame_np))
    data_frame_stair = np.cumsum(data_frame_bumps)

    # on calcule la largeur de chaque palier
    hist, _ = np.histogram(data_frame_stair, bins=max(data_frame_stair)+1)

    # on calcule les 2 moyennes à l'aide de la technique de k-moyennes
    k_means = scipy.cluster.vq.kmeans(hist.astype(float), 2)[0]
    k_means_sorted = np.sort(k_means)

    # on calcule le threshold à utiliser pour délimiter
    # les "grands" et "petits" bonds
    threshhold_value = np.average(k_means_sorted)

    # on compte la largeur de chaque étage de l'escalier
    largeurs = np.unique(data_frame_stair, return_counts=True)[1]

    # on threshold pour conntaire si c'est des grands ou petits bonds
    largeurs[largeurs < threshhold_value] = 0
    largeurs[largeurs >= threshhold_value] = 1
    largeurs = largeurs.tolist()

    # on retire les bordures
    largeurs.pop(0)
    largeurs.pop(len(largeurs)-1)

    # on trouve la fin d'un code, qui est toujours indiqué par la même
    # série de bonds,
    # soit ppppppppppppppg. D'ailleurs, cette série est impossible ailleurs

    string_pattern = ''.join(str(a) for a in largeurs)
    end_pattern_index = string_pattern.rfind(PATTERN_END)

    code = list()
    skip_next = False
    last_symbol = 1
    for jump in reversed(string_pattern[0:end_pattern_index]):

        if not skip_next:
            if jump == '1':
                last_symbol = 1-last_symbol
            elif jump == '0':
                skip_next = True
            code.append(last_symbol)
        else:
            skip_next = False

        # on a les 7 symboles
        if len(code) == 7:
            break

    code.reverse()
    code_str = ''.join(str(_) for _ in code)
    if len(code_str) == 7:
        return code_str
    else:
        return None
