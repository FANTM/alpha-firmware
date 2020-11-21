import json

NAME = "packet"

DATA_DEF_FILE    = '{}.json'.format(NAME)
HEADER_OUT_FILE  = '{}.h'.format(NAME)
SOURCE_OUT_FILE  = '{}.c'.format(NAME)

def get_data_type(data_item):
    ## Need some way to actually get key from schema!
    signed = data_item.get('signed', False)
    size = data_item.get('size')
    return "{}int{}_t".format('' if signed else 'u', size * 8)

def gen_packet_struct(data):
    output_string = "typedef struct Packet_t {\n"
    for item in data:
        data_type = get_data_type(item)
        output_string += "{} {};\n".format(data_type, item.get('name'))
    
    output_string += "} Packet_t;\n"
    return output_string

# Generalize to take any file name
def gen_open_header_guard():
    return "#ifndef __fantmpacket_h__\n#define __fantmpacket_h__\n\n"

def gen_close_header_guard():
    return "\n#endif\n"

def gen_packet_size_define(data):
    num_bytes = 0
    for item in data:
        num_bytes += item.get('size')
    return "#define PACKET_SIZE {}".format(num_bytes)

def get_byte(value, position):
    """Big endian"""
    return "0xff & (packet->{} >> {})".format(value, position * 8)

def gen_telemetry_send_declaration():
    return "void telemetrySend(Packet_t *data);"

def gen_telemetry_send_function(data):
    func_start = "void telemetrySend(Packet_t *packet) {"
    func_body  = """
    uint16_t size=PACKET_SIZE;
    uint8_t data[PACKET_SIZE];
    serializePacket(packet, data);
    writeBLE(data, &size);"""
    func_end   = "}"
    return "{}\n{}\n{}".format(func_start, func_body, func_end)

def gen_serialize_packet_declaration():
    return "static void serializePacket(Packet_t *packet, uint8_t *data);"

def gen_serialize_packet_function(data):
    """
    Using the struct format we define, creates a function that packs the data into a byte array
    """
    func_start = "static void serializePacket(Packet_t *packet, uint8_t *data) {\n"
    lines = ""
    func_end = "\n}\n"
    num_bytes = 0
    for item in data:
        size = item.get('size')
        while size > 0:
            size = size - 1
            lines += "data[{}] = {};\n".format(num_bytes, get_byte(item.get('name'), size))
            num_bytes = num_bytes + 1
    
    return "{}{}{}".format(
        func_start, 
        lines, 
        func_end)

def gen_include(name):
    return "#include \"{}\"\n".format(name)

def build_header_file(data):
    output_string = "{}\n{}\n{}\n{}\n{}\n{}".format(
        gen_open_header_guard(),
        gen_include("app_util_platform.h"),
        gen_packet_size_define(data), 
        gen_packet_struct(data),
        gen_telemetry_send_declaration(),
        gen_close_header_guard())
    print(output_string)
    return output_string
    
def build_source_file(data):
    output_string = "{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n".format(
        gen_include(HEADER_OUT_FILE),
        gen_include("fantmBLE.h"),
        gen_include("app_util_platform.h"),
        gen_include("nordic_common.h"),
        gen_include("boards.h"),
        gen_include("app_error.h"),
        gen_serialize_packet_declaration(),
        gen_serialize_packet_function(data),
        gen_telemetry_send_function(data)
    )
    print(output_string)
    return output_string

"""API"""

def get_data_model():
    with open(DATA_DEF_FILE) as packet_file:
        packet_json = json.load(packet_file)
    
    return packet_json

def write_header_file(data, path):
    contents = build_header_file(data)
    with open("{}{}".format(path, HEADER_OUT_FILE), "w") as out:
        out.write(contents)

def write_source_file(data, path):
    contents = build_source_file(data)
    with open("{}{}".format(path, SOURCE_OUT_FILE), "w") as out:
        out.write(contents)
    