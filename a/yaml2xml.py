
from pprint import pprint
import yaml
import sys
import yamale
import xmlplain
from xml.etree.ElementTree import fromstring
from collections import OrderedDict
from xmljson import BadgerFish, Yahoo, Parker, Abdera, Cobra, GData
from xmljson import XMLData
from json import dumps
import xmltodict
from pprint import pprint
import json
from xmljson import badgerfish as bf
import yaml
from pprint import pprint
from xmltodict import unparse
import xmltodict
from xmltodict import unparse
from lxml import etree


class MyClass(XMLData):
    def __init__(self, **kwargs):
        super(MyClass, self).__init__(attr_prefix='@', text_content='$t', simple_text=True, **kwargs)


def dict_to_xml(my_dict: dict):
    return unparse(my_dict, pretty=True)


def make_file(path, str_x):
    with open(path, 'w', encoding='utf-8') as w:
        w.write(str_x)


def yaml_to_xml(yaml_path):
    with open(yaml_path, 'r', encoding='utf-8') as file:
        fruits_list = yaml.load(file, Loader=yaml.FullLoader)
        mydict = dict(fruits_list)
        xx = unparse(mydict, pretty=True)
        return xx


def yaml_to_dict(yaml_path):
    with open(yaml_path, 'r', encoding='utf-8') as file:
        fruits_list = yaml.load(file, Loader=yaml.FullLoader)
        return dict(fruits_list)


def xml_to_dict(path_xml):
    with open(path_xml, 'r', encoding='utf-8') as r:
        ss = r.read()
        print(ss)
        return BadgerFish(dict_type=dict).data(fromstring(ss))


def validate(schema_path, xml_path):
    schema_root = etree.parse(schema_path)
    schema = etree.XMLSchema(schema_root)

    xml = etree.parse(xml_path)

    if not schema.validate(xml):
        print(schema.error_log)
    else:
        print('Валидация прошла успешно')


def dict_to_yaml(dict_x: dict):
    return yaml.dump(dict_x, allow_unicode=True, sort_keys=True)


def read_file(path):
    with open(path, 'r', encoding='utf-8') as r:
        return r.read()


if __name__ == "__main__": 


    path_yaml = sys.argv[1]
    path_xml = sys.argv[2]
    xml_str = yaml_to_xml(path_yaml)
    make_file(path_xml, xml_str)
