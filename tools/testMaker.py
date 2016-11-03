from random import randint
import re
from shutil import rmtree
import xml.etree.ElementTree as ET
from os import path, mkdir, remove, listdir
from subprocess import Popen, PIPE, STDOUT, TimeoutExpired
from collections import defaultdict

from jenks import jenks  # library with jenks natural breaks from https://github.com/perrygeo/jenks


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


def make_map(source_map_filename, output_filename, task):
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

    tree.write(output_filename)


def get_astar_nodes(Astar_exe_path, map_path, task, timeout=5, verbose=False):
    test_map = "/tmp/1.xml"
    while path.isfile(test_map):
        test_map = "/tmp/" + str(randint(1, 100000)) + ".xml"

    make_map(map_path, test_map, task)
    try:
        proc = Popen([Astar_exe_path, test_map], stdin=PIPE, stdout=PIPE,
                     stderr=STDOUT)
        out, _ = proc.communicate(timeout=timeout)
        # print(out.decode())
        nodes = nodes_pattern.search(out.decode()).group('nodes')
        remove(test_map)
        return int(nodes)
    except TimeoutExpired as e:
        proc.kill()
        if verbose:
            print(e)
    return None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", required=True, help="Путь к исполняемому файлу A*")
    parser.add_argument("--test", required=True, help="Путь к папке с картой в формате XML или единичному файлу")
    parser.add_argument("-n", required=False, help="Количество тестов на одной карте", type=int, default=30)
    parser.add_argument("-k", required=True, type=int, help="Количество групп тестов")
    parser.add_argument("--output", required=False, help="Путь для вывода результата", default="tests/")
    args = parser.parse_args()
    exec_path = path.abspath(args.exe)
    if not path.isfile(exec_path):
        print("Incorrect path to the executable")
        exit(1)
    dir_path = path.abspath(args.test)
    if path.isdir(dir_path):
        files = listdir(dir_path)
    elif path.isfile(dir_path):
        files = [dir_path]
        dir_path = path.dirname(dir_path)
    else:
        print("Incorrect path to the test")
        exit(1)

    # prepairing output dir
    output_dir = args.output
    if path.isdir(output_dir):
        rmtree(output_dir)
    if path.isfile(output_dir):
        remove(output_dir)
    mkdir(output_dir)

    tasks = defaultdict(list)
    nodes_pattern = re.compile(r'nodescreated=(?P<nodes>\d+).*')
    for map_name in files:
        try:
            data = parse_map(path.join(dir_path, map_name))
        except Exception:
            continue
        while len(tasks[map_name]) < args.n:
            task_s = [randint(0, data['width'] - 1), randint(0, data['height'] - 1), 0]
            task_f = [randint(0, data['width'] - 1), randint(0, data['height'] - 1), 0]
            task_s[2] = data['map'][task_s[1]][task_s[0]]
            task_f[2] = data['map'][task_f[1]][task_f[0]]
            if data['min_alt'] is not None:
                task_s[2] = max(task_s[2], data['min_alt'])
                task_f[2] = max(task_f[2], data['min_alt'])
            if data['max_alt'] is not None:
                task_s[2] = min(task_s[2], data['max_alt'])
                task_f[2] = min(task_f[2], data['max_alt'])

            task = {
                'start': tuple(task_s),
                'finish': tuple(task_f),
                'map': map_name,
                'nodes': 0
            }
            nodes = get_astar_nodes(exec_path, path.join(dir_path, map_name), task)
            if nodes is not None:
                task['nodes'] = nodes
                tasks[map_name].append(task)

    tmp = []
    for elem in tasks.values():
        tmp.extend(elem)
    tasks = tmp

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
    main_name_pattern = re.compile(r'.*/(?P<name>[^/]*)\.xml')
    for i in range(len(groups)):
        summary.write("Group " + str(i + 1) + ':\n\t')
        print(*groups[i], sep='\n\t', file=summary)
        mkdir(path.join(output_dir, str(i + 1)))
        for j in range(len(groups[i])):
            name = main_name_pattern.match(groups[i][j]['map']).group('name')
            make_map(groups[i][j]['map'],
                     path.join(path.join(output_dir, str(i + 1)), name + '_' + str(i + 1) + '_' + str(j + 1) + '.xml'),
                     groups[i][j])
