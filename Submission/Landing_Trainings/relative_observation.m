function obs = relative_observation(drone_state, platform_pos, platform_vel, height_offset)
% function used to evlauate the relative position to drone
obs = drone_state;
obs(1) = drone_state(1) - platform_pos(1);
obs(2) = drone_state(2) - platform_pos(2);
obs(3) = drone_state(3) - (platform_pos(3) + height_offset);
obs(4) = drone_state(4) - platform_vel(1);
obs(5) = drone_state(5) - platform_vel(2);
obs(6) = drone_state(6) - platform_vel(3);

end