import os
import sys
import shutil
import platform

def build_proto():
    # Build and source paths
    build_dir = 'build'
    proto_src_dir = 'api_docs/anx'

    # Clean build dir
    python_proto_build_dir = os.path.join(build_dir, "proto", "python")
    if os.path.exists(python_proto_build_dir):
        shutil.rmtree(python_proto_build_dir)
    cpp_proto_build_dir = os.path.join(build_dir, "proto", "cpp")
    if os.path.exists(cpp_proto_build_dir):
        shutil.rmtree(cpp_proto_build_dir)

    os.makedirs(os.path.join(build_dir, "proto", "python"))
    os.makedirs(os.path.join(build_dir, "proto", "cpp"))

    # List of proto files
    protos = [file for file in os.listdir(proto_src_dir) if file.split('.')[-1] == "proto"]
    print(f"protos: {protos}")

    # Get compiler
    protoc = None
    if platform.uname().system == 'Linux':
        if platform.uname().machine == 'x86_64':
            protoc = "protoc-22.0-linux-x86_64"
        elif platform.uname().machine == 'aarch64':
            protoc = "protoc-22.0-linux-aarch_64"
    elif platform.uname().system == 'Darwin':
        if platform.uname().machine == 'arm64':
            protoc = "protoc-22.0-osx-aarch_64"

    if protoc == None:
        sys.exit("No protoc found for your machine!!")

    # Build protos
    for proto in protos:
        os.system(f"./third_party/protoc/{protoc} --proto_path={proto_src_dir} --python_out={build_dir}/proto/python --cpp_out={build_dir}/proto/cpp {proto}")

    # Add __init__.py
    fd = open(os.path.join(build_dir, "proto", "python", "__init__.py"), "w")
    fd.close()

    # Replace import with from . import
    protos_pb2 = [f"{proto.split('.')[0]}_pb2" for proto in protos]
    for file in protos_pb2:
        fd = open(os.path.join(build_dir, "proto", "python", file + ".py") , "r")
        lines = fd.read().splitlines()
        for line_no, line in enumerate(lines):
            words = line.split(' ')
            if words[0] == "import":
                if words[1] in protos_pb2:
                    words[0] = "from . import"
                    line = ' '.join(words)
                    lines[line_no] = line
        fd.close()
        fd = open(os.path.join(build_dir, "proto", "python", file + ".py"), "w")
        fd.write("\n".join(lines))
        fd.close()

    # Create symlink
    proto_symlink_paths = ["./anx_interface/proto", "./anx_mock/proto"]
    for proto_symlink_path in proto_symlink_paths:
        if os.path.exists(proto_symlink_path):
            os.remove(proto_symlink_path)
        os.symlink(f"../{build_dir}/proto/python", proto_symlink_path)

# Main
build_proto()
