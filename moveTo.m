function moveTo(target, current, leftMotor, rightMotor, gyro)
    direction = target - current;
    angle = atan2d(direction(2), direction(1));
    

    if angle < 0
        angle = angle + 360;
    end
    
    resetRotationAngle(gyro);
    
    
    while abs(readRotationAngle(gyro) - angle) > 5
        leftMotor.Speed = -20;
        rightMotor.Speed = 20;
        start(leftMotor);
        start(rightMotor);
    end
    stop(leftMotor);
    stop(rightMotor);
    
    
    leftMotor.Speed = 30;
    rightMotor.Speed = 30;
    start(leftMotor);
    start(rightMotor);
    pause(1); 
    stop(leftMotor);
    stop(rightMotor);
end
