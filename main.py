from classes import Arranger
from classes import Judge
import sys

S = Judge()
sys.argv = ['0', '/Users/mac/Desktop/2019HuaWei/1-map-training-2/car.txt',
            '/Users/mac/Desktop/2019HuaWei/1-map-training-2/road.txt',
            '/Users/mac/Desktop/2019HuaWei/1-map-training-2/cross.txt',
            '/Users/mac/Desktop/2019HuaWei/1-map-training-2/answer.txt']

car_path = sys.argv[1]
road_path = sys.argv[2]
cross_path = sys.argv[3]
answer_path = sys.argv[4]
S.get_car_info(car_path)
S.get_cross_info(cross_path)
S.get_road_info(road_path)

a = Arranger(S)