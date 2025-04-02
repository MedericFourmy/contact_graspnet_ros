# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators



class contact_graspnet_ros:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        z_range = [0.2, 2.0]
        local_regions = True
        filter_grasps = True
        skip_border_objects = True
        forward_passes = 1



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = contact_graspnet_ros.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("contact_graspnet_ros." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "z_range":
                    validation_result = ParameterValidators.fixed_size(param, 2)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.z_range = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "local_regions":
                    updated_params.local_regions = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "filter_grasps":
                    updated_params.filter_grasps = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "skip_border_objects":
                    updated_params.skip_border_objects = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "forward_passes":
                    validation_result = ParameterValidators.gt_eq(param, 0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.forward_passes = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "z_range"):
                descriptor = ParameterDescriptor(description="Z value threshold to crop the input point cloud.", read_only = False)
                parameter = updated_params.z_range
                self.node_.declare_parameter(self.prefix_ + "z_range", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "local_regions"):
                descriptor = ParameterDescriptor(description="Crop 3D local regions around given segments.", read_only = False)
                parameter = updated_params.local_regions
                self.node_.declare_parameter(self.prefix_ + "local_regions", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "filter_grasps"):
                descriptor = ParameterDescriptor(description="Filter grasp contacts according to the segmentation masks.", read_only = False)
                parameter = updated_params.filter_grasps
                self.node_.declare_parameter(self.prefix_ + "filter_grasps", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "skip_border_objects"):
                descriptor = ParameterDescriptor(description="When extracting local_regions, ignore segments at depth map boundary.", read_only = False)
                parameter = updated_params.skip_border_objects
                self.node_.declare_parameter(self.prefix_ + "skip_border_objects", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "forward_passes"):
                descriptor = ParameterDescriptor(description="Run multiple parallel forward passes to mesh_utils more potential contact points/grasps.", read_only = False)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 2**31-1
                parameter = updated_params.forward_passes
                self.node_.declare_parameter(self.prefix_ + "forward_passes", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "z_range")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.fixed_size(param, 2)
            if validation_result:
                raise InvalidParameterValueException('z_range',param.value, 'Invalid value set during initialization for parameter z_range: ' + validation_result)
            updated_params.z_range = param.value
            param = self.node_.get_parameter(self.prefix_ + "local_regions")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.local_regions = param.value
            param = self.node_.get_parameter(self.prefix_ + "filter_grasps")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.filter_grasps = param.value
            param = self.node_.get_parameter(self.prefix_ + "skip_border_objects")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.skip_border_objects = param.value
            param = self.node_.get_parameter(self.prefix_ + "forward_passes")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt_eq(param, 0)
            if validation_result:
                raise InvalidParameterValueException('forward_passes',param.value, 'Invalid value set during initialization for parameter forward_passes: ' + validation_result)
            updated_params.forward_passes = param.value


            self.update_internal_params(updated_params)
