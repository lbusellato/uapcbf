import functools
import rclpy.logging

from jaka_interface.exceptions import *

#########################################
#                                       #
# Definitions for the decorators used   #
# in the JakaInterface class            #
#                                       #
#########################################

def process_sdk_call(connected: bool=False, 
                    powered_on: bool=False, 
                    enabled: bool=False, 
                    servo_enabled: bool=False,
                    servo_disabled: bool=False,
                    update_context: str=None,
                    update_attr: str=None,
                    update_mode: int=0,
                    update_val: bool=None,
                    success_log_msg: str=None):
    """Decorator to validate SDK calls, checking both preconditions and handling return values.
        
    Parameters
    ----------
    connected : bool, optional
        If True, requires that self.is_connected is True, by default False
    powered_on : bool, optional
        If True, requires that self.state.is_powered_on is True, by default False
    enabled : bool, optional
        If True, requires that self.state.is_enabled is True, by default False
    servo_enabled : bool, optional
        If True, requires that self.state.is_in_servo_mode is True, by default False
    servo_disabled : bool, optional
        If True, requires that self.state.is_in_servo_mode is False, by default False

    Returns
    -------
        The wrapped function only if all specified conditions are met; otherwise, it logs a warning and returns None. 
        The function returns the return value of the SDK call, if no error code was received, or None if no return value 
        was received.

    Raises
    ------
    JakaInterfaceException
    """
    
    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            logger = rclpy.logging.get_logger(str(self))
            context = func.__name__

            # Go over the preconditions
            if servo_enabled and not self.state.is_in_servo_mode:
                logger.warning(f"{context}: Servo mode must be enabled to perform this action.")
                return None
            elif servo_disabled and self.state.is_in_servo_mode:
                logger.warning(f"{context}: Servo mode must be disabled to perform this action.")
                return None
            elif enabled and not self.state.is_enabled:
                logger.warning(f"{context}: Robot must be enabled to perform this action.")
                return None
            elif powered_on and not self.state.is_powered_on:
                logger.warning(f"{context}: Robot must be powered on to perform this action.")
                return None
            elif connected and not self.state.is_connected:
                logger.warning(f"{context}: Robot must be connected to perform this action.")
                return None
            
            ret = func(self, *args, **kwargs)
            error_code = ret[0]
            
            # Check if the SDK returned an error
            if error_code != JAKA_ERR_CODES.SUCCESS_CODE.value: 
                error_message = f"[{context}]: {JAKA_ERR_MSGS.get(error_code)}"
                logger.error(error_message)
                exc_class: JakaInterfaceException = ERROR_EXCEPTION_MAP.get(error_code)
                raise exc_class(error_message)
            
            # Check if the user wants to carry out updates to the state variables
            if update_context is not None: 
                context = getattr(self, update_context) if update_context else self
                if update_mode == 0: # Apply argument
                    val = args[0]
                elif update_mode == 1: # Invert argument
                    val = not getattr(context, update_attr)
                elif update_mode == 2: # Apply return value
                    val = ret[1]
                else: # Apply custom value
                    val = update_val                
                setattr(context, update_attr, val)

            # Log the user given success message, if any
            if success_log_msg: logger.info(success_log_msg)
            
            return ret[1] if len(ret) > 1 else None
        return wrapper
    return decorator

def untested(func):
    """Decorator to warn the user about untested functions with a log message (only once not to hinder performance).
        
    Returns
    -------
        The wrapped function.
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not wrapper.has_run:
            logger = rclpy.logging.get_logger(str(self))
            context = func.__name__
            logger.warning(f"{context} is untested, so its behaviour/safety is not guaranteed. If you confirm its \
                           functionality, please remove the @untested decorator from it and open a PR on the repo.")
            wrapper.has_run = True
            return func(self, *args, **kwargs)
    wrapper.has_run = False     
    return wrapper

def deprecated(func):
    """Decorator to warn the user about deprecated functions with a log message (only once not to hinder performance).
        
    Returns
    -------
        The wrapped function.
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not wrapper.has_run:
            context = func.__name__
            rclpy.logging.get_logger(str(self)).warning(f"{context} is deprecated after controller version 1.7.1, so it\
                                                         might not work in the future. Refer to \
                                                        https://www.jaka.com/docs/en/guide/SDK/API%20Change.html#deprecated-interfaces.")
            wrapper.has_run = True
            return func(self, *args, **kwargs)
    wrapper.has_run = False     
    return wrapper