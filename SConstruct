import os
import sys
import shutil
import platform

def build_proto():
    # Build and source paths
    build_dir = 'anx_proto'
    proto_src_dir = 'api_docs/anx'

    # Clean build dir
    python_proto_build_dir = os.path.join(build_dir, "python")
    if os.path.exists(python_proto_build_dir):
        shutil.rmtree(python_proto_build_dir)
    cpp_proto_build_dir = os.path.join(build_dir, "cpp")
    if os.path.exists(cpp_proto_build_dir):
        shutil.rmtree(cpp_proto_build_dir)

    os.makedirs(os.path.join(build_dir, "python"))
    os.makedirs(os.path.join(build_dir, "cpp"))

    # List of proto files
    protos = [file for file in os.listdir(proto_src_dir) if file.split('.')[-1] == "proto"]
    print(f"protos: {protos}")

    # Get compiler
    protoc_version = "3.12.4"
    protoc = None
    if platform.uname().system == 'Linux':
        if platform.uname().machine == 'x86_64':
            protoc = f"protoc-{protoc_version}-linux-x86_64"
        elif platform.uname().machine == 'aarch64':
            protoc = f"protoc-{protoc_version}-linux-aarch_64"
    elif platform.uname().system == 'Darwin':
        if platform.uname().machine == 'x86_64':
            protoc = f"protoc-{protoc_version}-osx-x86_64"
        elif platform.uname().machine in ['arm64','aarch64'] :
            protoc = f"protoc-{protoc_version}-osx-aarch_64"

    if protoc == None:
        sys.exit("No protoc found for your machine!!")

    # Build protos
    for proto in protos:
        os.system(f"./third_party/protoc/{protoc} --proto_path={proto_src_dir} --python_out={build_dir}/python --cpp_out={build_dir}/cpp {proto}")

    # Add __init__.py
    fd = open(os.path.join(build_dir, "python", "__init__.py"), "w")
    fd = open(os.path.join(build_dir, "__init__.py"), "w")
    fd.close()

    # Replace import with from . import
    protos_pb2 = [f"{proto.split('.')[0]}_pb2" for proto in protos]
    for file in protos_pb2:
        fd = open(os.path.join(build_dir, "python", file + ".py") , "r")
        lines = fd.read().splitlines()
        for line_no, line in enumerate(lines):
            words = line.split(' ')
            if words[0] == "import":
                if words[1] in protos_pb2:
                    words[0] = "from . import"
                    line = ' '.join(words)
                    lines[line_no] = line
        fd.close()
        fd = open(os.path.join(build_dir, "python", file + ".py"), "w")
        fd.write("\n".join(lines))
        fd.close()

# Main
build_proto()
