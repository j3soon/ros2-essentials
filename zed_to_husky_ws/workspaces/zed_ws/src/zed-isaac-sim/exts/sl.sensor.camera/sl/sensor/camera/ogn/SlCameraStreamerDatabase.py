"""Support for simplified access to data on nodes of type sl.sensor.camera.ZED_Camera

Streams ZED camera data to the ZED SDK
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import traceback
import carb
import sys
class SlCameraStreamerDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type sl.sensor.camera.ZED_Camera

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.camera_prim
            inputs.exec_in
            inputs.serial_number
            inputs.streaming_port
            inputs.use_system_time
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:camera_prim', 'bundle', 0, 'ZED Camera prim', 'ZED Camera prim used to stream data', {}, True, None, False, ''),
        ('inputs:exec_in', 'execution', 0, 'ExecIn', 'Triggers execution', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:serial_number', 'uint', 0, 'Serial number', 'Serial number (identification) of the camera to stream, can be left to default. It must be of one of the compatible values: 20976320, 29123828, 25626933, 27890353, 25263213, 21116066, 27800035, 27706147', {ogn.MetadataKeys.DEFAULT: '20976320'}, True, 20976320, False, ''),
        ('inputs:streaming_port', 'uint', 0, 'Streaming port', 'Streaming port - unique per camera', {ogn.MetadataKeys.DEFAULT: '30000'}, True, 30000, False, ''),
        ('inputs:use_system_time', 'bool', 0, 'Use system time', 'Override simulation time with system time for image timestamps', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
    ])
    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.inputs.camera_prim = og.Database.ROLE_BUNDLE
        role_data.inputs.exec_in = og.Database.ROLE_EXECUTION
        return role_data
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"exec_in", "serial_number", "streaming_port", "use_system_time", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.__bundles = og.BundleContainer(context, node, attributes, [], read_only=True, gpu_ptr_kinds={})
            self._batchedReadAttributes = [self._attributes.exec_in, self._attributes.serial_number, self._attributes.streaming_port, self._attributes.use_system_time]
            self._batchedReadValues = [0, 20976320, 30000, False]

        @property
        def camera_prim(self) -> og.BundleContents:
            """Get the bundle wrapper class for the attribute inputs.camera_prim"""
            return self.__bundles.camera_prim

        @property
        def exec_in(self):
            return self._batchedReadValues[0]

        @exec_in.setter
        def exec_in(self, value):
            self._batchedReadValues[0] = value

        @property
        def serial_number(self):
            return self._batchedReadValues[1]

        @serial_number.setter
        def serial_number(self, value):
            self._batchedReadValues[1] = value

        @property
        def streaming_port(self):
            return self._batchedReadValues[2]

        @streaming_port.setter
        def streaming_port(self, value):
            self._batchedReadValues[2] = value

        @property
        def use_system_time(self):
            return self._batchedReadValues[3]

        @use_system_time.setter
        def use_system_time(self, value):
            self._batchedReadValues[3] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues
    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }
    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = SlCameraStreamerDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = SlCameraStreamerDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = SlCameraStreamerDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(SlCameraStreamerDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'sl.sensor.camera.ZED_Camera'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = SlCameraStreamerDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = SlCameraStreamerDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = SlCameraStreamerDatabase(node)

            try:
                compute_function = getattr(SlCameraStreamerDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return SlCameraStreamerDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            SlCameraStreamerDatabase._initialize_per_node_data(node)
            initialize_function = getattr(SlCameraStreamerDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(SlCameraStreamerDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            SlCameraStreamerDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(SlCameraStreamerDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(SlCameraStreamerDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "sl.sensor.camera")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "ZED camera streamer")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "Stereolabs")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORY_DESCRIPTIONS, "Stereolabs,Nodes used with the Stereolabs ZED SDK")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Streams ZED camera data to the ZED SDK")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                SlCameraStreamerDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(SlCameraStreamerDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        SlCameraStreamerDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(SlCameraStreamerDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("sl.sensor.camera.ZED_Camera")
