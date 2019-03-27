import sys
import time
from classes import Judge

if __name__ == '__main__':
    start_time = time.time()
    S = Judge()
    sys.argv = ['0', '../train1/car.txt', '../train1/road.txt', '../train1/cross.txt', '../train1/answer.txt']
    [car_file, road_file, cross_file, plan_file] = sys.argv[1:5]
    #    print([car_file, road_file, cross_file, plan_file])
    # [car_file, road_file, cross_file, plan_file] = sys.argv[1:5]

    # 解析文件
    S.parse_Plan_info(plan_file)
    S.get_car_info(car_file)
    S.get_cross_info(cross_file)
    S.get_road_info(road_file)

    result = 0  # 系统调度时间
    tick = -1  # 车辆运行时间

    active_stack = []  # 存放当前时刻应被处理的车辆编号
    active_road = []  # 存放当前时刻有车的道路编号
    final_stack = []
    while True:
        tick += 1  # 表往后走一秒
        # 当前未处理车辆，车到终点后弹出
        dead_stack = []  # 储存本时刻已经被处理的车辆
        # 规则一：系统先调度在行驶的车辆，再调度等待上路的车辆
        garage_stack = sorted(S.StartTimeBin.get(tick, []))  # 车库里将要上路的车
        # 规则二：按照车辆ID升序进行调度
        # 第一步：道路调度，进行一次
        # print('Arrange Roads...')
        for road_id in S.road_No:
            # print('Now processing road No.', road_id)
            # print(S.road_info[5014].WhichCar())
            road = S.road_info[road_id]
            car_list, rev_car_list = road.WhichCar()
            # print(car_list, rev_car_list)
            for car_id in car_list:
                # print(car_id)
                dead_stack, S.car_info[car_id], signal = road.DriveCar(S.car_info[car_id], dead_stack, S)
            for car_id in rev_car_list:
                dead_stack, S.car_info[car_id], signal = road.DriveCar(S.car_info[car_id], dead_stack, S, reversed=True)
        # 道路调度后，调度时间+1
        result += 1
        # 第二步：路口调度，直到所有车辆被标记为final
        # print('Arrange Cross...')
        while len(active_stack) != len(dead_stack):
            # if result == 155:
            #     print(0)
            handled_signal = 0
            # 此时意味着车道上的车辆全部处理完了,继续使车库里的车上路
            for cross_id in S.cross_No:
                # print('Now processing cross No.', cross_id)
                # if cross_id == 31:
                #     print(0)
                cross = S.cross_info[cross_id]
                # print('Arrange cross No {}'.format(cross.id))
                # handled=True表示有车的状态发生变化
                tmp1 = len(dead_stack)
                tmp2 = handled_signal
                dead_stack, S, handled = cross.crossArrange(dead_stack, S)
                handled_signal |= handled
            if len(active_stack) != len(dead_stack):
                assert handled_signal, 'Dead Lock Happened!'
            result += 1  # 每次路口的循环需要调度时间+1
        # 第三步：车库发车
        # print('Place Car from garage...')
        for car_id in garage_stack:
            # print('Now place car No.', car_id)
            car = S.car_info[car_id]
            dead_stack, S, signal = car.PlaceCar_FromGarage(S, dead_stack)
            if signal:
                active_stack.append(car.id)
        result += 1
        # 道路调度完成，此时判断各个车辆是否到达其终点
        # 若到达终点，从active_stack中剔除，若未到达终点，将其state刷新为wait
        for car_id in dead_stack:
            car = S.car_info[car_id]
            if car.reachFinal(S):
                # print('No {} car reached final.'.format(car.id))
                active_stack.remove(car_id), final_stack.append(car_id)
            else:
                car.reflush()
        # 如果时刻超过了planTime的最大值且没有未到终点的车辆，意味着时间结束
        if tick >= max(S.time) and not active_stack:
            break

    print('''Totol arrange time is {} tick, the program run time is {} s.'''.format(tick + 16,
                                                                                    round(time.time() - start_time, 2)))
