#!/usr/bin/python3

from jsonschema import validate
from helpers import get_data_model, write_source_file, write_header_file
import json
import argparse


def verify():
    with open('schema.json') as schema_file:
        schema = json.load(schema_file)
    with open('packet.json') as packet_file:
        packet = json.load(packet_file) 
    
    is_valid = validate(instance=packet, schema=schema)
    print("Valid packet")  
    

def main(only_verify, path):
    verify()
    if only_verify:
        return
    
    data = get_data_model()
    write_header_file(data, path)
    write_source_file(data, path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Verify and build a single source of truth schema used for bluetooth communication')
    parser.add_argument('-v', "--verify", help="Just verify the validity of the schema", action="store_true")
    parser.add_argument('-p', "--path", help="relative path to where you want the files to be output (without file name)", default="./")
    args = parser.parse_args()

    main(args.verify, args.path)