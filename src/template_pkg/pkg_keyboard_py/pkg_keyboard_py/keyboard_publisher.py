#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import threading

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher_node')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info('Keyboard Publisher Node Started. Press keys to publish. Press Esc to exit.')
        
        self.running = True
        self._pynput_listener_instance = None # To store the listener instance

        # daemon=True means this thread won't block program exit if main thread finishes,
        # but we join it explicitly for a clean shutdown.
        self.listener_thread = threading.Thread(target=self._keyboard_listen, daemon=True, name="KeyboardListenerThread")
        self.listener_thread.start()

    def on_press(self, key):
        # If shutdown has started (self.running is False), stop listening.
        # Also, if rclpy is not ok, we should stop.
        if not self.running or not rclpy.ok():
            return False # Returning False stops the pynput listener

        try:
            # Try to get the character representation of the key
            key_char = key.char
            if key_char:
                msg = String()
                msg.data = key_char
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: "{msg.data}"')
        except AttributeError:
            # Handle special keys (e.g., Shift, Ctrl, Alt, Esc)
            if key == keyboard.Key.esc:
                self.get_logger().info('Esc pressed, initiating node shutdown...')
                self.running = False  # Signal main loop and listener to terminate
                return False  # Stop the pynput listener from its own callback
            # You can publish special key names if needed
            # msg = String()
            # msg.data = f"SPECIAL_KEY_{key}"
            # self.publisher_.publish(msg)
            # self.get_logger().info(f'Publishing: "{msg.data}"')
            pass # For other special keys, do nothing for now
        
        # Continue listening if self.running is True, otherwise stop.
        return self.running

    def _keyboard_listen(self):
        self.get_logger().debug(f'{threading.current_thread().name} started.')
        try:
            # The 'with' statement ensures the listener is started and properly stopped/joined.
            # pynput.Listener runs in its own internal thread(s).
            # This current thread (_keyboard_listen) will block on l.join() until the listener stops.
            with keyboard.Listener(on_press=self.on_press) as l:
                self._pynput_listener_instance = l # Store instance for external stop
                l.join()  # Blocks until the listener is stopped
        except Exception as e:
            # Log exceptions that occur during listener setup or if join fails,
            # not usually exceptions from on_press (pynput often handles those).
            self.get_logger().error(f"Exception in _keyboard_listen: {e}", exc_info=True)
        finally:
            self._pynput_listener_instance = None # Clear the stored instance
            self.get_logger().debug(f'{threading.current_thread().name} finished.')
            # Ensure self.running is False so main loop also exits if listener thread stops unexpectedly.
            self.running = False

    def _initiate_shutdown(self):
        """Initiates a graceful shutdown of the node and its components."""
        if not self.running and self._pynput_listener_instance is None and (not self.listener_thread or not self.listener_thread.is_alive()):
            # Already shutdown or in the process from another path
            return
            
        self.get_logger().info('Initiating node shutdown sequence...')
        self.running = False # Primary flag to stop all loops and activities

        # Stop the pynput listener if it's active and referenced
        if self._pynput_listener_instance:
            self.get_logger().debug('Requesting pynput listener to stop.')
            self._pynput_listener_instance.stop()
        
        # Wait for the listener thread to finish
        if self.listener_thread and self.listener_thread.is_alive():
            self.get_logger().debug(f"Waiting for {self.listener_thread.name} to join...")
            self.listener_thread.join(timeout=1.5) # Give it some time to exit
            if self.listener_thread.is_alive():
                self.get_logger().warn(f'{self.listener_thread.name} did not join in time.')
        else:
            self.get_logger().debug('Listener thread already finished or was not started.')

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    
    try:
        # Main loop, kept alive by keyboard_publisher.running and rclpy.ok()
        while rclpy.ok() and keyboard_publisher.running:
            # Spin once to allow ROS communications and responsiveness to shutdown.
            rclpy.spin_once(keyboard_publisher, timeout_sec=0.1)
    except KeyboardInterrupt:
        keyboard_publisher.get_logger().info('KeyboardInterrupt (Ctrl+C) received.')
    finally:
        keyboard_publisher.get_logger().info('Main loop exited. Cleaning up node...')
        keyboard_publisher._initiate_shutdown() # Gracefully stop listener thread and other activities
        
        # Check if node is still valid before destroying
        # The node's context should be ok and its public handle should not be None
        if keyboard_publisher.context.ok() and keyboard_publisher.handle is not None:
             keyboard_publisher.get_logger().debug('Destroying node...')
             keyboard_publisher.destroy_node()
        else:
            keyboard_publisher.get_logger().debug('Node already destroyed or context invalid.')
        
        if rclpy.ok(): # Check if rclpy context is still valid globally
            keyboard_publisher.get_logger().debug('Shutting down rclpy...')
            rclpy.shutdown()
        keyboard_publisher.get_logger().info('ROS 2 shutdown complete.')

if __name__ == '__main__':
    main()