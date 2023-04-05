#!/usr/bin/env python3

from enum import Enum
import numpy as np
import zmq

import anx_proto.python.common_pb2 as common_pb2
import anx_proto.python.model_pb2 as model_pb2

tflite_numpy_dtype_map = {
    1: np.float32,
    2: np.int32,
    3: np.uint8,
    4: np.int64,
    7: np.int16,
    9: np.int8,
    10: np.float16,
    11: np.float64,
    13: np.uint64,
    16: np.uint32
}

class DeviceType(Enum):
    CPU = 1
    GPU = 2
    DSP = 3

class Tensor():
    def __init__(self):
        self.tensor = None
        self.shape = None
        self.dtype = None

class TfliteInterface:
    def __init__(self, device_type):
        """
        Arguments:
            device_type -- select from DeviceType enum
        """
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REQ)

        if device_type == DeviceType.CPU:
            self.socket.connect(f"ipc:///ipc/cpu")
        elif device_type == DeviceType.GPU:
            self.socket.connect(f"ipc:///ipc/gpu")
        elif device_type == DeviceType.DSP:
            self.socket.connect(f"ipc:///ipc/dsp")

        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.model_loaded = False
        self.model = None

        self.input_tensors = []

        self.output_tensors = []

    def load_model(self, path_to_model):
        """
        load tflite model
        Arguments:
            path_to_model -- path to the tflite model
        Returns:
            True if successful
        """

        with open(path_to_model, mode="rb") as model_fd:
            self.model = model_fd.read()
            request = common_pb2.Payload()
            request.payload = self.model
            request_bytes = request.SerializeToString()
            self.socket.send_multipart([b"LoadModel", request_bytes])

            events = self.poller.poll(10000)
            if events:
                model_meta_rep = model_pb2.ModelMeta()
                std_rep = common_pb2.StdResponse()
                std_rep_bytes, model_meta_rep_bytes = self.socket.recv_multipart()
                std_rep.ParseFromString(std_rep_bytes)
                if std_rep.success:
                    self.model_loaded = True
                    model_meta_rep.ParseFromString(model_meta_rep_bytes)

                    # Populate input/output shape & dtype
                    for index, input_tensor in enumerate(model_meta_rep.input_tensors):
                        self.input_tensors.append(Tensor())
                        self.input_tensors[-1].shape = tuple(input_tensor.dims)
                        self.input_tensors[-1].dtype = tflite_numpy_dtype_map[input_tensor.dtype]
                        print(f"Input{index} shape: {self.input_tensors[-1].shape}, dtype: {self.input_tensors[-1].dtype}")

                    for index, output_tensor in enumerate(model_meta_rep.output_tensors):
                        self.output_tensors.append(Tensor())
                        self.output_tensors[-1].shape = tuple(output_tensor.dims)
                        self.output_tensors[-1].dtype = tflite_numpy_dtype_map[output_tensor.dtype]
                        print(f"Output{index} shape: {self.output_tensors[-1].shape}, dtype: {self.output_tensors[-1].dtype}")

                    return True
                else:
                    print(std_rep.message)
        return False

    def unload_model(self):
        """
        unload model
        Returns:
            True if successful
        """
        request = common_pb2.Empty()
        request_bytes = request.SerializeToString()
        self.socket.send_multipart([b"UnloadModel", request_bytes])

        events = self.poller.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self.socket.recv()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.model_loaded = False
                self.model = None

                self.input_tensor = []

                self.output_tensor = []
                return True
            else:
                print(rep.message)
        return False

    def set_inputs(self, inputs):
        """
        set inputs to the model
        Arguments:
            inputs -- list of numpy array of shape and type as acceptable by tflite model
        Returns:
            True if successful
        """
        if not self.model_loaded:
            print("Load model first!!")
            return False

        # Check if input length is equal to self.input_sensor len
        if not len(inputs) == len(self.input_tensors):
            print("Expected inputs of length {len(self.input_tensors)}, but got {len(inputs)}")
            return False

        for index, input_ in enumerate(inputs):
            if not input_.shape == self.input_tensors[index].shape:
                print(f"Input{index} shape should be {self.input_tensor.shape}")
                return False

            if not input_.dtype == self.input_tensors[index].dtype:
                print(f"Input{index} dtype should be {self.input_tensor.dtype}")
                return False

            self.input_tensors[index].tensor = input_

        return True

    def invoke(self):
        """
        invoke the model
        Returns:
            True if successful
        """

        if not self.model_loaded:
            print("Load model first!!")
            return False

        req = common_pb2.PayloadArray()
        for input_tensor in self.input_tensors:
            req.payloads.append(input_tensor.tensor.tobytes())
        req_bytes = req.SerializeToString()
        self.socket.send_multipart([b"InvokeModel", req_bytes])

        events = self.poller.poll(5000)
        if events:
            rep = common_pb2.PayloadArray()
            rep_bytes = self.socket.recv()
            rep.ParseFromString(rep_bytes)
            
            for index, payload in enumerate(rep.payloads):
                self.output_tensors[index].tensor = np.frombuffer(payload, dtype=self.output_tensors[index].dtype)
                self.output_tensors[index].tensor = self.output_tensors[index].tensor.reshape(self.output_tensors[index].shape)
            return True
        return False

    def get_output(self):
        """
        Returns:
            model output as numpy array 
        """
        if not self.model_loaded:
            print("Load model first!!")
            return None
        return [output_tensor.tensor for output_tensor in self.output_tensors]
