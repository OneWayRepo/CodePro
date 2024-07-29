#! /usr/bin/env python3
# coding=utf-8

"""
 description:   json file operation
 author:		kevin.wang
 create date:	2024-04-22
 version:		1.0.0
"""


import os
import pathlib
import json


class JsonMrg():
    """
    docstring for JsonMrg
    """
    def __init__(self, filepath):
        self.dict = {}
        self.filepath = filepath

    def __del__(self):
        pass

    def set_path(self, path):
        self.filepath = path

    def set_data(self, dict):
        self.dict = dict

    def get_data(self):
        return self.dict

    def clear_data(self):
        self.dict = {}
        
    def load_data_from_file(self) -> bool:
        try:
            if self.filepath != '':
                with open(self.filepath, 'r') as fp:
                    self.dict = json.load(fp)
                return True
        except Exception as e:
            print(e)
            return False

    def dump_data_to_json(self) -> bool:
        p = pathlib.Path(self.filepath)
        if p.exists():
            os.remove(self.filepath)
        with open(self.filepath, 'w', encoding='utf-8') as fp:
            json.dump(self.dict, fp, ensure_ascii=False)


if __name__ == '__main__':
    print("hello the world")
