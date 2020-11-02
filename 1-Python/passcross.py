def passfunc(self, car, next_lane, next_road, four_road, cross_id, dead_stack, S: Judge, direction: int, s2: int, lane, reversed=False):
    if car.state == 'wait':
        next_pos = s2 - 1
        next_state = 1
        flag = 0
        handled = 0
        straight_road_id = left_road_id = -1
        if direction in [1, 3]:
            straight_road_id = four_road[(four_road.index(next_road.id) - 2) % 4]
        if direction == 3:
            left_road_id = four_road[(four_road.index(next_road.id) - 1) % 4]
        if straight_road_id != -1:
            straight_road = S.road_info[straight_road_id]
            straight_car_list, straight_rev_car_list = straight_road.WhichCar()
        if left_road_id != -1:
            left_road = S.road_info[left_road_id]
            left_car_list, left_rev_car_list = left_road.WhichCar()
        # 处理直行道路是否逆行道路
        if direction in [1, 3] and straight_road_id != -1:
            if straight_road.end != cross_id:
                straight_todo_list = deepcopy(straight_rev_car_list)
            else:
                straight_todo_list = deepcopy(straight_car_list)
        else:
            straight_todo_list = []
        # 处理左拐道路是否逆行道路
        if direction == 3 and left_road_id != -1:
            if left_road.end != cross_id:
                left_todo_list = deepcopy(left_rev_car_list)
            else:
                left_todo_list = deepcopy(left_car_list)
        else:
            left_todo_list = []
        if s2 <= 0:
            lane[car.lane][car.road_pos] = 0
            car.road_pos = self.length - 1  # 到达车道最前端
            lane[car.lane][car.road_pos] = car
            car.state = 'final'
            dead_stack.append(car.id)
            next_state = 0
            flag == 2
        elif flag == 0:
            for car_id in straight_todo_list:
                if S.car_info[car_id].state == 'wait' and S.car_info[car_id].RoadSequence[
                    S.car_info[car_id].road_index + 1] == next_road.id:
                    flag = 1
                    break
                if S.car_info[car_id].state == 'wait' and S.car_info[car_id].RoadSequence[
                    S.car_info[car_id].road_index + 1] != next_road.id:
                    flag = 0
                    break
            if not flag:
                for car_id in left_todo_list:
                    if S.car_info[car_id].state == 'wait' and S.car_info[car.id].RoadSequence[
                        S.car_info[car_id].road_index + 1] == next_road.id:
                        flag = 1
                        break
                    if S.car_info[car_id].state == 'wait' and S.car_info[car.id].RoadSequence[
                        S.car_info[car_id].road_index + 1] != next_road.id:
                        flag = 0
                        break
            for i in range(next_road.channel):
                tmp = -1
                for j in range(next_pos + 1):
                    if next_lane[i][j] != 0 and next_lane[i][j].state == 'wait':
                        # 表示前进道路上有等待的车，自己也等待，后续不能开flag=1
                        next_state = 0
                        flag = 1
                        break
                    elif next_lane[i][j] and next_lane[i][j].state == 'final':
                        tmp = j - 1  # 最近的一辆final的车，如果能走，走到它后面，如果不能，进入下一个车道
                        break
                    elif next_lane[i][j] == 0:
                        if j >= tmp:
                            tmp = j  # tmp记录在最小车道上车辆能到达的最远的位置
                if tmp >= 0:
                    next_state = 0
                    lane[car.lane][car.road_pos] = 0
                    next_lane[i][tmp] = car
                    car.lane = i
                    car.road_pos = tmp
                    car.road_index += 1
                    car.state = 'final'
                    dead_stack.append(car.id)
                    S.car_info[car.id] = car
                    flag = 2
                    break
                if flag:
                    break
        if next_state:
            handled = 1
            lane[car.lane][car.road_pos] = 0
            lane[car.lane][-1] = car
            car.road_pos = self.length - 1
            car.state = 'final'
            dead_stack.append(car.id)
            S.car_info[car.id] = car
            flag = 2
        if flag == 2:
            handled = 1
            # 当成功调度路口后，马上调度这条车道上所有的车。
            for j in range(self.length - 1, -1, -1):
                if lane[lane_prev][j] != 0:
                    dead_stack, car, signal = self.DriveCar(lane[lane_prev][j], dead_stack, S, reversed)
                    S.car_info[car.id] = car
        if flag == 1:  # 表示未能成功通行，其后的所有车都无法通行。
            handled = 0
            return dead_stack, handled
    return dead_stack, handled
