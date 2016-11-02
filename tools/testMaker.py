from random import randint
import re
import xml.etree.ElementTree as ET
from os import path, mkdir
from subprocess import Popen, PIPE, STDOUT


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


if __name__ == "__main__":
    map_filename = input("Enter path to the map:    ")
    map_filename = path.abspath(map_filename)
    data = parse_map(map_filename)
    number_of_taskes = int(input("Enter number of tasks to generate:    "))
    clusters_number = int(input('Enter number of test groups to create:    '))

    taskes = []

    # print(*taskes, sep='\n')
    results = []
    nodes_pattern = re.compile(r'nodescreated=(?P<nodes>\d+).*')
    while len(taskes) < number_of_taskes:
        output_filename = '/tmp/test.xml'
        open(output_filename, 'w').close()
        task_s = [randint(0, data['width'] - 1), randint(0, data['height'] - 1), 0]
        task_f = [randint(0, data['width'] - 1), randint(0, data['height'] - 1), 0]
        task_s[2] = data['map'][task_s[1]][task_s[0]]
        task_f[2] = data['map'][task_f[1]][task_f[0]]
        task = {
            'start': tuple(task_s),
            'finish': tuple(task_f)
        }
        # print(task)
        make_map(map_filename, output_filename, task)

        try:
            proc = Popen([path.join(path.dirname(__file__), "Astar"), output_filename], stdin=PIPE, stdout=PIPE,
                         stderr=STDOUT)
            out, _ = proc.communicate(timeout=4)
            # print(out.decode())
            nodes = nodes_pattern.search(out.decode()).group('nodes')
            task['nodescreated'] = int(nodes)
            taskes.append(task)
            results.append(int(nodes))
        except Exception as e:
            print(e)

    edges = []
    for i in range(len(taskes)):
        for j in range(i + 1, len(taskes)):
            edges.append((i, j, abs(results[i] - results[j])))
    edges.sort(key=lambda e: e[2])
    # print(*edges)
    clusters = DSU(len(taskes))

    i = 0
    while clusters.sets_number > clusters_number:
        clusters.join(edges[i][0], edges[i][1])
        i += 1

    test_groups = []
    for cluster in clusters.sets():
        group = []
        for i in cluster:
            group.append(taskes[i])
        test_groups.append(group)

    main_name = re.match(r'.*/(?P<name>[^/]*)\.xml', map_filename).group('name')
    mkdir('tests')
    f = open('tests/summary.txt', 'w')
    for i in range(len(test_groups)):
        f.write('\ngroup ' + str(i + 1) + ':\n\t')
        print(*test_groups[i], sep='\n\t', file=f)

        mkdir('tests/' + str(i + 1))
        for j in range(len(test_groups[i])):
            make_map(map_filename,
                     "tests/" + str(i + 1) + '/' + main_name + '_' + str(i + 1) + '_' + str(j + 1) + '.xml',
                     test_groups[i][j])

    f.close()
