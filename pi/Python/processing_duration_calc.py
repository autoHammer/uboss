from numpy import sqrt
import os


def get_avg(path):
    file = open(path, "r")
    len_ = 0
    sum_ = 0

    for line in file.readlines():
        sum_ = sum_ + float(line)
        len_ = len_ + 1

    file.close()
    return sum_ / len_


def get_variance(path, avg):
    file = open(path, "r")
    len_ = 0
    variance = 0

    for line in file.readlines():
        variance = variance + (float(line) - avg) ** 2
        len_ = len_ + 1

    file.close()
    return variance / (len_ - 1)


if __name__ == '__main__':
    SCRIPT_PATH = os.path.abspath(__file__)
    SCRIPT_DIR = os.path.dirname(SCRIPT_PATH) + "/logs"

    custom_traps_nano = SCRIPT_DIR + "/custom_traps_nano_2024-05-01_13-18-44.txt"
    custom_traps_small = SCRIPT_DIR + "/custom_traps_small_2024-05-01_13-24-44.txt"
    traps_only_small = SCRIPT_DIR + "/traps_only_small_2024-05-01_13-29-21.txt"
    v8_small_first_best = SCRIPT_DIR + "/v8_small_first_best_2024-05-01_13-37-09.txt"

    custom_traps_nano_avg = get_avg(custom_traps_nano)
    custom_traps_nano_variance = get_variance(custom_traps_nano, custom_traps_nano_avg)

    print("custom traps nano varians: " + str(custom_traps_nano_variance))
    print("custom traps nano standardavvik: " + str(sqrt(custom_traps_nano_variance)))
    print("custom traps nano gjennomsnitt: " + str(custom_traps_nano_avg) + "\n")

    custom_traps_small_avg = get_avg(custom_traps_small)
    custom_traps_small_variance = get_variance(custom_traps_small, custom_traps_small_avg)

    print("custom traps small varians: " + str(custom_traps_small_variance))
    print("custom traps small standardavvik: " + str(sqrt(custom_traps_small_variance)))
    print("custom traps small gjennomsnitt: " + str(custom_traps_small_avg) + "\n")

    traps_only_small_avg = get_avg(traps_only_small)
    traps_only_small_variance = get_variance(traps_only_small, traps_only_small_avg)

    print("traps only small varians: " + str(traps_only_small_variance))
    print("traps only small standardavvik: " + str(sqrt(traps_only_small_variance)))
    print("traps only small gjennomsnitt: " + str(traps_only_small_avg) + "\n")

    v8_small_first_best_avg = get_avg(v8_small_first_best)
    v8_small_first_best_variance = get_variance(v8_small_first_best, v8_small_first_best_avg)

    print("v8 first small varians: " + str(v8_small_first_best_variance))
    print("v8 first small standardavvik: " + str(sqrt(v8_small_first_best_variance)))
    print("v8 first small gjennomsnitt: " + str(v8_small_first_best_avg) + "\n")
