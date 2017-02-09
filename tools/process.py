#! /usr/bin/env python3
import multiprocessing
import subprocess
import fractions
import xml.etree.ElementTree as ET

import re
from PIL import Image, ImageDraw, ImageFont

SETTINGS = {
    'EMPTY_COLOR': (255, 255, 255),
    'PATH_COLOR': (0, 0, 255),
    'OBSTACLE_COLOR': (0, 0, 0),
    'START_COLOR': (0, 255, 0),
    'FINISH_COLOR': (255, 0, 0),
    'CLOSED_COLOR': (153, 88, 61),
    'OPENED_COLOR': (204, 74, 20),
    'TEXT_COLOR': (0, 0, 0),
}


def parse_log(filename, shell=False, is_section_path=None):
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
        parse_result["cellsize"] = int(map.find("cellsize").text)
    except AttributeError:
        print("Couldn't find size of cell (attribute <cellsize>) in {}. Would be ignored.".format(filename))

    parse_result['start'] = (int(map.find("startx").text), int(map.find("starty").text), int(map.find("startz").text))
    parse_result['finish'] = (int(map.find("finishx").text), int(map.find("finishy").text), int(map.find("finishz").text))

    parse_result['map'] = []
    for plain in map.find('grid'):
        parse_result['map'].append([int(i) for i in plain.text.split()])

    if parse_result['max_level'] is None:
        parse_result['max_level'] = max(chain.from_iterable(parse_result['map']))

    parse_result['floor'] = None
    parse_result['ceiling'] = None

    try:
        alt_lims = map.find('altitudelimits')
        parse_result['floor'] = int(alt_lims.get('min'))
        parse_result['ceiling'] = int(alt_lims.get('max'))
    except Exception:
        pass

    if parse_result['floor'] is None:
        parse_result['floor'] = 0

    if parse_result['ceiling'] is None:
        parse_result['ceiling'] = parse_result['max_level']


    algo_name = root.find('algorithm').find('searchtype').text.lower()
    if algo_name in {'bfs', 'jp_search', 'dijkstra', 'astar'}:
        any_angle_search = False
    else:
        any_angle_search = True

    log = root.find('log')

    path = []
    closed = set()
    opened = set()

    if is_section_path is None:
        is_section_path = False

    if log.find('path').text != 'Path NOT found!':
        level = log.find('lplevel')
        # For all paths 'lppath' section is used
        if (not is_section_path and level is not None):
            path = set()
            for node in level.iter('node'):
                path.add((int(node.get('x')), int(node.get('y'))))
        else:
            parse_result['section_path'] = True
            level = log.find('hplevel')
            section = level.find('section')
            path.append((int(section.get('start.x')), int(section.get('start.y'))))
            for section in level.iter('section'):
                path.append((int(section.get('finish.x')), int(section.get('finish.y'))))

        level = log.find('viewed')
        if level is None:
            if shell:
                print("Can not find viewed section. Points visited by algorithm won't be shown for {}.".format(filename))
        else:
            for node in level.iter('node'):
                if (node.get('closed')):
                    closed.add((int(node.get('x')), int(node.get('y'))))
                else:
                    opened.add((int(node.get('x')), int(node.get('y'))))

    parse_result['section_path'] = False
    parse_result['closed_list'] = closed
    parse_result['opened_list'] = opened
    parse_result['path'] = path

    summary = log.find('summary')
    parse_result['summary'] = {}
    parse_result['summary']['path_found'] = (summary.get('pathfound') == 'true')
    if parse_result['summary']['path_found']:
        parse_result['summary']['lenth'] = float(summary.get('pathlength'))
        parse_result['summary']['time'] = float(summary.get('time'))
        parse_result['summary']['steps'] = int(summary.get('numberofsteps'))
        parse_result['summary']['nodes'] = int(summary.get('nodescreated'))
        if algo_name == 'lian':
            parse_result['summary']['max_angle'] = float(summary.get('maxAngle'))
            parse_result['summary']['sections'] = int(summary.get('sections'))

    return parse_result


def illustrate(parsed_data, output_filename, output_format="PNG", scale=2):
    scale = int(scale)
    scale = 1 if scale == 0 else scale
    height = parsed_data['height']
    width = parsed_data['width']
    start = parsed_data['start']
    finish = parsed_data['finish']

    text_zone_height = 0

    im = Image.new("RGB", (scale * width, scale * (height + text_zone_height)), color=SETTINGS['EMPTY_COLOR'])

    dr = ImageDraw.Draw(im)

    for x in range(width):
        for y in range(height):
            if (x, y) in parsed_data['opened_list']:
                color = SETTINGS['OPENED_COLOR']
            elif (x, y) in parsed_data['closed_list']:
                color = SETTINGS['CLOSED_COLOR']
            elif (x, y) == start:
                color = SETTINGS["START_COLOR"]
            elif (x, y) == finish:
                color = SETTINGS["FINISH_COLOR"]
            elif not parsed_data['section_path'] and (x, y) in parsed_data['path']:
                color = SETTINGS['PATH_COLOR']
            elif parsed_data['map'][y][x]:
                if parsed_data['max_level'] is not None:
                    color = [0] * len(SETTINGS['OBSTACLE_COLOR'])
                    for i in range(len(SETTINGS['OBSTACLE_COLOR'])):
                        color[i] = SETTINGS['EMPTY_COLOR'][i] + parsed_data['map'][y][x] * (
                        SETTINGS['OBSTACLE_COLOR'][i] - SETTINGS['EMPTY_COLOR'][i]) / parsed_data['max_level']
                        color[i] = int(color[i])
                    color = tuple(color)

                else:
                    color = SETTINGS['OBSTACLE_COLOR']
            else:
                color = SETTINGS['EMPTY_COLOR']

            dr.rectangle([(scale * x, scale * y), (scale * x + scale, scale * y + scale)], fill=color)

    if parsed_data['section_path']:
        dr.line(list(map(lambda elem: (scale * elem[0], scale * elem[1]), parsed_data['path'])),
                fill=SETTINGS['PATH_COLOR'], width=scale)
    del dr
    im.save(output_filename, output_format)


def parse_and_illustrate(log_filename, picture_filename=None, output_format="PNG", scale=2):
    data = parse_log(log_filename)
    if picture_filename is None:
        m = re.match(r'(?P<name>.+)_log\.xml', log_filename)
        picture_filename = path.join(path.dirname(log_filename), m.group('name') + ".plain.png")
    illustrate(data, picture_filename, output_format, scale)


def get_log_output_filename(task_filename):
    tree = ET.parse(task_filename)
    root = tree.getroot()
    logpath = root.find('options').find('logpath')
    path = logpath.text
    if path is not None:
        return path
    m = re.match(r'(?P<name>.+)(?<!_log)\.xml', task_filename)
    return m.group('name') + "_log.xml"


def make_path(exec_filename, input_filename, timeout=40):
    subprocess.run([exec_filename, input_filename],
                   stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, timeout=timeout)


def make_path_and_picture(exec_filename, input_filename, log_filename, picture_filename, scale=2, timeout=15, picture_format='PNG'):
    code = subprocess.run([exec_filename, input_filename],
                          stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, timeout=timeout).returncode
    if code == 0:
        data = parse_log(log_filename)
        illustrate(data, picture_filename, picture_format, scale)
        print("{} processed successfully".format(input_filename))
    else:
        print("Error has occurred during searching path for {}.\
        Program has finished with exit code {}. Trying to process log file, if it exists.".format(input_filename, code))
        try:
            data = parse_log(log_filename)
            illustrate(data, picture_filename, picture_format, scale)
        except Exception as e:
            print("Processing log file {} is failed with error".format(log_filename), e, sep='\n')


if __name__ == "__main__":
    import argparse
    from os import listdir, cpu_count, path, walk
    from time import time

    parser = argparse.ArgumentParser()
    parser.add_argument("--exe", required=True, help="Путь к исполняемому файлу проекта")
    parser.add_argument("--test", required=True, help="Путь к папке с заданиями в формате XML или единичному файлу")
    parser.add_argument("--logs", required=False, help="Путь к папке с результатами поиска в формате XML или единичному файлу")
    parser.add_argument("--scale", required=False, type=int,
                        help="Масштаб карты, относительно заданного в файле", default=2)
    parser.add_argument("--no_image", required=False, help="Не генерировать изображения к результату", action="store_true")
    parser.add_argument('-r', required=False, help="Рекурсивно искать задания", action="store_true")
    parser.add_argument('-t', required=False, help="Количество потоков выполнения заданий", type=int, default=4)
    args = parser.parse_args()
    exec_path = path.abspath(args.exe)
    if not path.isfile(exec_path):
        print("Incorrect path to the executable")
        exit(1)

    task_pattern = re.compile(r'(.*/)?(?P<name>[^/]+)(?<!_log)\.xml')
    dir_path = path.abspath(args.test)
    if path.isdir(dir_path):
        task_files = []
        if args.r:
            for root, dirs, files in walk(dir_path):
                for file in files:
                    if task_pattern.match(file):
                        task_files.append(path.abspath(path.join(root, file)))
        else:
            for file in listdir(dir_path):
                if task_pattern.match(file):
                    task_files.append(path.abspath(path.join(dir_path, file)))
    elif path.isfile(dir_path):
        task_files = [path.abspath(dir_path)]
        dir_path = path.dirname(dir_path)
    else:
        print("Incorrect path to the test")
        exit(1)

    logs = None
    if args.logs:
        logs_path = path.abspath(args.logs)
        if path.isdir(logs_path):
            logs = []
            for log_name in listdir(logs_path):
                logs.append(path.join(logs_path, log_name))
        elif path.isfile(logs_path):
            logs = [logs_path]
        else:
            print("Incorrect path to the logs")
            exit(1)

    tasks = {}
    for filename in task_files:
        m = task_pattern.match(filename)
        if m or len(task_files) == 1:
            tasks[filename] = (get_log_output_filename(filename),
                                                    path.join(path.dirname(filename), m.group('name') + '.plain.png'))

    start_time = time()
    with multiprocessing.Pool(min(args.t, cpu_count())) as pool:
        if logs is not None:
            logs_pattern = re.compile(r'(?P<name>.+)_log\.xml')
            for cand in logs:
                if logs_pattern.match(cand) or len(logs) == 1:
                    pool.apply_async(parse_and_illustrate, [cand])
        else:
            for inp_path, val in tasks.items():
                if args.no_image:
                    pool.apply_async(make_path, [exec_path, inp_path])
                else:
                    pool.apply_async(make_path_and_picture, [exec_path, inp_path, val[0], val[1], args.scale])
        pool.close()
        pool.join()
    print('Finished in {} seconds'.format(time() - start_time))

