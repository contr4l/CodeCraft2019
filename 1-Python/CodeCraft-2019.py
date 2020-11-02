import logging
import sys
import Judge
import time
import multiprocessing

logging.basicConfig(level=logging.DEBUG,
                    filename='../logs/CodeCraft-2019.log',
                    format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S',
                    filemode='a')


def main():
    if len(sys.argv) != 5:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]

    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("answer_path is %s" % (answer_path))
    Judge.start(car_path, road_path, cross_path, answer_path)


def search_params(running_list_bound=50, count_bound=200):
    if len(sys.argv) != 5:
        logging.info('please input args: car_path, road_path, cross_path, answerPath')
        exit(1)

    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]

    logging.info("car_path is %s" % (car_path))
    logging.info("road_path is %s" % (road_path))
    logging.info("cross_path is %s" % (cross_path))
    logging.info("answer_path is %s" % (answer_path))
    Judge.start(car_path, road_path, cross_path, answer_path, running_list_bound, count_bound)
# to read input file
# process
# to write output file


def test(running_list_bound):
        try:
            print('Test running_list_bound {}..'.format(running_list_bound))
            search_params(running_list_bound)
        except AssertionError:
            print('running_list_bound {} deadlock..'.format(running_list_bound))

if __name__ == "__main__":
    start_time = time.time()
    main()
    print(time.time()-start_time, 's')
    # pool = multiprocessing.Pool(processes=multiprocessing.cpu_count())
    # pool.map(test, range(50, 70))

