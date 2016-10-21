import unittest
import epics
import numpy as np
import time

nx, ny = 256, 768
P = 'djv:'
R = 'cam1:'
I = 'image1:'

acquire_mode = {
    'done'   : 0,
    'acquire': 1
}

data_types = {
    'int8'  : 0,
    'uint8'   : 1,
    'int16' : 2,
    'uint16'  : 3,
    'int32' : 4,
    'uint32'  : 5,
    'float32': 6,
    'float64': 7
}

color_modes = {
    'mono'  : 0,
    'bayer' : 1,
    'RGB1'  : 2,
    'RGB2'  : 3,
    'RGB3'  : 4,
    'YUV444': 5,
    'YUV422': 6,
    'YUV421': 7
}

array_modes = {
    'overwrite': 0,
    'append'   : 1
}

image_modes = {
    'single': 0,
    'multiple': 1,
    'continuous' :2
}

trigger_modes = {
    'internal': 0,
    'external': 1
}

array_callbacks = {
    'disable': 0,
    'enable' : 1
}

image_callbacks = {
    'disable': 0,
    'enable' : 1
}
class TestUInt8(unittest.TestCase):
    
    def setUp(self):
        print('Begin setup')
        self.image = np.random.uniform(0, 255, size=nx*ny).astype(np.uint8).reshape(nx, ny)
        self.acquire_pv = epics.PV(P+R+'Acquire.VAL')
        self.image_plugin_array_pv = epics.PV(P+I+'ArrayData.VAL')
        self.image_callbacks_pv = epics.PV(P+I+'EnableCallbacks.VAL')
        self.size_x_pv = epics.PV(P+R+'SizeX.VAL')
        self.size_y_pv = epics.PV(P+R+'SizeY.VAL')
        self.data_type_pv = epics.PV(P+R+'DataType.VAL')
        self.color_mode_pv = epics.PV(P+R+'ColorModde.VAL')
        self.num_images_pv = epics.PV(P+R+'NumImages.VAL')
        self.array_mode_pv = epics.PV(P+R+'ArrayMode.VAL')
        self.num_elements_pv = epics.PV(P+R+'NumElements.VAL')
        self.image_mode_pv = epics.PV(P+R+'ImageMode.VAL')
        self.trigger_mode_pv = epics.PV(P+R+'TriggerMode.VAL')
        self.detector_state_pv = epics.PV(P+R+'DetectorState_RBV.VAL')
        self.array_callbacks_pv = epics.PV(P+R+'ArrayCallbacks.VAL')
        self.array_in_pv = epics.PV(P+R+'ArrayInUInt8.VAL')
        print('PVs connected')

        self.size_x_pv.put(nx)
        self.size_y_pv.put(ny)
        self.data_type_pv.put(data_types['uint8'])
        self.color_mode_pv.put(color_modes['mono'])
        self.num_images_pv.put('5')
        self.array_mode_pv.put(array_modes['overwrite'])
        self.num_elements_pv.put(100)
        self.image_mode_pv.put(image_modes['single'])
        self.image_callbacks_pv.put(image_callbacks['enable'])
        self.trigger_mode_pv.put(trigger_modes['internal'])
        self.array_callbacks_pv.put(array_callbacks['enable'])
        print('Completed setup')

    def tearDown(self):
        pass

    def test_single_acquisition(self):
        # Test overwrite mode
        self.array_mode_pv.put(array_modes['overwrite'])
        self.acquire_pv.put(acquire_mode['acquire'])
        time.sleep(1.0)
        self.array_in_pv.put(self.image.flatten(), wait=True)
        image = self.image_plugin_array_pv.get(count=nx*ny)
        print(self.image.flatten()[300:310])
        print(image[300:310])
        self.assertTrue(np.array_equal(image, self.image.flatten()))


if __name__ == '__main__':
    unittest.main()
