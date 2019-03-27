# -*- encoding=utf8 -*-
import sys
import time
from classes import Judge, Arranger


# if __name__ == '__main__':
def start(car_file, road_file, cross_file, answer_file):
    S = Judge()
    # sys.argv = ['0', 'train1/car.txt', 'train1/road.txt', 'train1/cross.txt', 'train1/answer.txt']
    # [car_file, road_file, cross_file, plan_file] = sys.argv[1:5]
    #    print([car_file, road_file, cross_file, plan_file])
    # [car_file, road_file, cross_file, plan_file] = sys.argv[1:5]

    # 解析文件
    # S.parse_Plan_info(answer_file)
    S.get_car_info(car_file)
    S.get_cross_info(cross_file)
    S.get_road_info(road_file)
    # S.simulate()
    A = Arranger(S)
    A.arrange()
    A.write_answers(answer_file)
