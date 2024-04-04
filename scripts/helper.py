import json
import os


def get_array_fields(filename, fields):
    file = open(filename, "r")
    lines = file.readlines()
    content = ''
    for line in lines:
        content += line

    array_buffer = json.loads(content)

    for field in fields:
        array_buffer = array_buffer[field]

    data = {}

    for i in range(len(array_buffer)):
        topic = array_buffer[i]['key']
        value = array_buffer[i]['value']
        data[topic] = value

    return data


def get_files_in_dir(dir, format):
    file_paths = []
    for folder, subs, files in os.walk(dir):
        for filename in files:
            file_paths.append(os.path.abspath(os.path.join(folder, filename)))
    files = [file for file in file_paths if file.endswith(format)]
    return files


def sort_param_files(files):
    sorted_files = sorted(files, key=lambda s: int(s[s.rfind('_') + 1:s.rfind('.')]))
    return sorted_files
