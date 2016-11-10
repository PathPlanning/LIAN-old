from random import randint, randrange
import re
from shutil import rmtree, copyfile
import xml.etree.ElementTree as ET
from os import path, mkdir, remove, listdir, chdir
from json import loads, load
from subprocess import Popen, PIPE, STDOUT, TimeoutExpired
from collections import defaultdict
from time import time

from jenks import jenks  # library with jenks natural breaks from https://github.com/perrygeo/jenks


EASIEST_TASK = 1000


class DSU:
    def __init__(self, size):
        self.data = [[i, 1] for i in range(size)]
        self.sets_number = size
        self._size = size

    def find(self, node):
        if self.data[node][0] == node:
            return node
        leader = self.find(self.data[node][0])
        self.data[node][0] = leader
        return leader

    def are_joined(self, x, y):
        return self.find(x) == self.find(y)

    def join(self, x, y):
        leader_x = self.find(x)
        leader_y = self.find(y)
        if leader_x == leader_y:
            return False

        if self.data[leader_x][1] > self.data[leader_y][1]:
            self.data[leader_y][0] = leader_x
            self.data[leader_x][1] += self.data[leader_y][1]
        else:
            self.data[leader_x][0] = leader_y
            self.data[leader_y][1] += self.data[leader_x][1]
        self.sets_number -= 1
        return True

    def size(self):
        return self._size

    def sets(self):
        used = set()
        for i in range(len(self.data)):
            if self.find(i) not in used:
                res = [j for j in range(len(self.data)) if self.are_joined(i, j)]
                used.add(self.find(i))
                yield res

def parse_map(filename):
    parse_result = {}

    tree = ET.parse(filename)
    root = tree.getroot()
    map = root.find('map')
    try:
        parse_result["title"] = map.find('title').text
    except AttributeError:
        parse_result["title"] = None
    parse_result["width"] = int(map.find("width").text)
    parse_result["height"] = int(map.find("height").text)
    try:
        parse_result['max_level'] = int(map.find("maxaltitude").text)
    except AttributeError:
        parse_result["max_level"] = None
        print("Couldn't find <maxaltitude> attribute image will be monochromatic")

    try:
        altlims = map.find('altitudelimits')
        parse_result['min_alt'] = int(altlims.get('min'))
        parse_result['max_alt'] = int(altlims.get('max'))
    except AttributeError:
        parse_result['min_alt'] = None
        parse_result['max_alt'] = None

    parse_result['map'] = []
    for plain in map.find('grid'):
        parse_result['map'].append([int(i) for i in plain.text.split()])

    return parse_result


def make_map(source_map_filename, output_filename, task, algorithm_params_json=None):
    tree = ET.parse(source_map_filename)
    root = tree.getroot()
    map = root.find('map')

    try:
        startx = map.find('startx')
        startx.text = str(task['start'][0])
    except AttributeError:
        startx = ET.Element('startx')
        startx.text = str(task['start'][0])
        map.append(startx)

    try:
        starty = map.find('starty')
        starty.text = str(task['start'][1])
    except AttributeError:
        starty = ET.Element('starty')
        starty.text = str(task['start'][1])
        map.append(starty)

    try:
        startz = map.find('startz')
        startz.text = str(task['start'][2])
    except AttributeError:
        startz = ET.Element('startz')
        startz.text = str(task['start'][2])
        map.append(startz)

    try:
        finishx = map.find('finishx')
        finishx.text = str(task['finish'][0])
    except AttributeError:
        finishx = ET.Element('finishx')
        finishx.text = str(task['finish'][0])
        map.append(finishx)

    try:
        finishy = map.find('finishy')
        finishy.text = str(task['finish'][1])
    except AttributeError:
        finishy = ET.Element('finishy')
        finishy.text = str(task['finish'][1])
        map.append(finishy)

    try:
        finishz = map.find('finishz')
        finishz.text = str(task['finish'][2])
    except AttributeError:
        finishz = ET.Element('finishz')
        finishz.text = str(task['finish'][2])
        map.append(finishz)

    if algorithm_params_json is not None:
        f = open(algorithm_params_json)
        params = load(f)
        f.close()
        try:
            algo = root.find('algorithm')
            algo.clear()
        except AttributeError:
            algo = ET.Element('algorithm')
            root.append(algo)
        for elem in params['algorithm']:
            tag = ET.SubElement(algo, elem[0])
            tag.text = elem[1]

    tree.write(output_filename)


def generate_test_for_map(map_path, number_tests, exe_path, sample_params=None, shared_list=None):
    res = []
    try:
        data = parse_map(map_path)
        j = map_path.rfind('/')
        map_name = map_path[j + 1:]
    except Exception:
        return
    while len(res) < number_tests:
        task_s = [randrange(0, data['width']), randrange(0, data['height']), 0]
        task_f = [randrange(0, data['width']), randrange(0, data['height']), 0]

        if data['max_alt'] is not None:
            task_s[2] = randint(data['map'][task_s[1]][task_s[0]], data['max_alt'])
            task_f[2] = randint(data['map'][task_f[1]][task_f[0]], data['max_alt'])
        else:
            task_s[2] = data['map'][task_s[1]][task_s[0]]
            task_f[2] = data['map'][task_f[1]][task_f[0]]
        if data['min_alt'] is not None:
            task_s[2] = max(task_s[2], data['min_alt'])
            task_f[2] = max(task_f[2], data['min_alt'])

        task = {
            'start': tuple(task_s),
            'finish': tuple(task_f),
            'map': map_name,
            'nodes': 0,
            'time': 0,
            'found': False
        }
        if sample_params is None:
            get_astar_summary(exe_path, map_path, task)
        else:
            get_astar_summary(exe_path, map_path, task, sample_json=sample_params)
        if task['found'] and task['nodes'] >= EASIEST_TASK:
            res.append(task)

    if shared_list is not None:
        shared_list.extend(res)
    else:
        return res


# tries to execute Astar binary. If successfully, write extra data to the task
def get_astar_summary(Astar_exe_path, map_path, task, timeout=5, verbose=False, sample_json=None):
    test_map = "/tmp/1.xml"
    while path.isfile(test_map):
        test_map = "/tmp/" + str(randint(1, 100000)) + ".xml"

    make_map(map_path, test_map, task, sample_json)
    try:
        proc = Popen([Astar_exe_path, test_map], stdin=PIPE, stdout=PIPE,
                     stderr=STDOUT)
        out, _ = proc.communicate(timeout=timeout)
        # print(out.decode())
        task['nodes'] = int(nodes_pattern.search(out.decode()).group('nodes'))
        task['time'] = float(time_pattern.search(out.decode()).group('time'))
        task['found'] = (out.decode().lower().find('path found') != -1)
        return True
    except TimeoutExpired as e:
        proc.kill()
        if verbose:
            print(e)
    except Exception:
        pass
    remove(test_map)
    return False

nodes_pattern = re.compile(r'nodescreated=(?P<nodes>\d+)')
time_pattern = re.compile(r'time=(?P<time>\d+\.\d*)')
main_name_pattern = re.compile(r'.*/?(?P<name>[^/]*)\.xml')
group_title = re.compile(r'group (?P<number>\d+):?\s*', re.IGNORECASE)


if __name__ == "__main__":
    import argparse
    from multiprocessing import Pool, Manager

    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", required=True, help="Путь к исполняемому файлу A*")
    parser.add_argument("--test", required=True, help="Путь к папке с картой в формате XML или единичному файлу")
    parser.add_argument("-n", required=True, help="Количество тестов на одной карте", type=int)
    parser.add_argument("-k", required=False, type=int, help="Количество групп тестов", default=1)
    parser.add_argument("--output", required=False, help="Путь для вывода результата", default="tests/")
    parser.add_argument("--write_maps", required=False, help="Генерировать карты заданий", default=False, type=bool)
    parser.add_argument("--log", required=False, help="Взять результаты теста из указанного файла")
    args = parser.parse_args()
    exec_path = path.abspath(args.exe)
    if not path.isfile(exec_path):
        print("Incorrect path to the executable")
        exit(1)
    map_dir = path.abspath(args.test)
    if path.isdir(map_dir):
        files = listdir(map_dir)
    elif path.isfile(map_dir):
        j = map_dir.rfind('/')
        files = [map_dir]
        map_dir = path.dirname(map_dir)
        files[0] = files[0][j + 1:]
    else:
        print("Incorrect path to the test")
        exit(1)

    # Getting params of algorithms if possible
    if 'algorithm_params' in listdir(path.dirname(__file__)):
        json_pattern = re.compile(r'(?P<filename>.*)\.json')
        params = {}
        for file in listdir(path.join(path.dirname(__file__), 'algorithm_params')):
            if json_pattern.match(file):
                params[json_pattern.match(file).group('filename')] = path.abspath(path.join(
                    path.join(path.dirname(__file__), 'algorithm_params'), file))
        if '_sample' not in params:
            params = None
            print("Please specify '_sample.json'. This file is needed for constructing tests. ")
    else:
        params = None
        print("Couldn't find 'algorithm_params' folder. Will be generated tests only for algorithm from given maps.")

    start_time = time()
    # Parsing previous results
    if args.log:
        f = open(args.log)
        tasks = []
        borders = [0, 0]
        lines = f.readlines()
        f.close()
        for i, row in enumerate(lines):
            if group_title.match(row.strip()):
                borders[0] = i
                break
        try:
            borders[1] = lines.index('\n')
        except ValueError:
            borders[1] = len(lines)
        for i, row in enumerate(lines):
            if borders[0] < i < borders[1] and not group_title.match(row.strip()):
                tasks.append(eval(row.strip()))
    else:
        tasks = None

    # prepairing output dir
    output_dir = path.abspath(args.output)
    if path.isdir(output_dir):
        rmtree(output_dir)
    if path.isfile(output_dir):
        remove(output_dir)
    mkdir(output_dir)

    if tasks is None:
        manager = Manager()
        pool = Pool(processes=4)
        tasks = manager.list()
        for map_name in files:
            pool.apply_async(generate_test_for_map, (path.join(map_dir, map_name), args.n, exec_path, params['_sample'], tasks))
        pool.close()
        pool.join()

    # Clustering tests and forming a groups
    keys = list(map(lambda x: x['nodes'], tasks))
    dividers = jenks(keys, args.k)
    print(dividers)

    groups = [[] for i in range(args.k)]
    for task in tasks:
        left = 0
        right = len(dividers) - 1
        while (right - left) > 1:
            mid = (right + left) // 2
            if dividers[mid] > task['nodes']:
                right = mid
            else:
                left = mid
        groups[left].append(task)

    summary = open(path.join(output_dir, "summary.txt"), 'w')

    mkdir(path.join(output_dir, 'source_maps'))
    for map_name in files:
        copyfile(path.join(map_dir, map_name), path.join(output_dir, 'source_maps/' + map_name))

    if params is None:
        for i in range(len(groups)):
            summary.write("Group " + str(i + 1) + ':\n\t')
            print(*groups[i], sep='\n\t', file=summary)
            if args.write_maps:
                mkdir(path.join(output_dir, str(i + 1)))
                for j in range(len(groups[i])):
                    name = main_name_pattern.match(groups[i][j]['map']).group('name')
                    make_map(groups[i][j]['map'],
                             path.join(path.join(output_dir, str(i + 1)), name + '_' + str(i + 1) + '_' + str(j + 1) + '.xml'),
                             groups[i][j])
    else:
        summary.write('TEST GROUPS:\n')
        for i in range(len(groups)):
            summary.write("Group " + str(i + 1) + ':\n\t')
            print(*sorted(groups[i], key=lambda x: x['nodes']), sep='\n\t', file=summary)

        if args.write_maps:
            del params['_sample']
            summary.write("\n\nALGORITHMS:\n")
            chdir(output_dir)
            for algo_name in params:
                summary.write('\t' + algo_name + ': ')
                f = open(params[algo_name])
                summary.write(loads(f.read())['description'] + '\n')
                f.close()

                mkdir(algo_name)
                chdir(algo_name)
                for i in range(len(groups)):
                    mkdir(str(i + 1))
                    for j in range(len(groups[i])):
                        name = main_name_pattern.match(groups[i][j]['map']).group('name')
                        make_map(path.join(map_dir, groups[i][j]['map']),
                                 path.join(str(i + 1), name + '_' + str(i + 1) + '_' + str(j + 1) + '.xml'),
                                 groups[i][j], params[algo_name])
                chdir(path.pardir)

    print("Finished in {} seconds".format(time() - start_time))
    summary.close()
