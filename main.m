myev3 = legoev3('wifi',"EVFOUR"); 
leftMotor = motor(myev3, 'B');   
rightMotor = motor(myev3, 'C');  
gyro = gyroSensor(myev3, 2);     
color = colorSensor(myev3, 3);   
proximity = ultrasonicSensor(myev3, 4); 


gridSize = [5, 5]; 
startPos = [1, 1];
goalPos = [5, 5];
obstacles = []; 


heuristic = @(pos) norm(pos - goalPos);


openList = {startPos};
cameFrom = containers.Map;
gScore = containers.Map;
fScore = containers.Map;
key = @(pos) sprintf('%d,%d', pos(1), pos(2));

gScore(key(startPos)) = 0;
fScore(key(startPos)) = heuristic(startPos);

while ~isempty(openList)
    
    [~, idx] = min(cellfun(@(p) fScore(key(p)), openList));
    current = openList{idx};
    
    if isequal(current, goalPos)
        break; 
    end
    
    openList(idx) = []; 
    
    
    directions = [0 1; 1 0; 0 -1; -1 0]; 
    for i = 1:size(directions,1)
        neighbor = current + directions(i,:);
        if any(neighbor < 1) || any(neighbor > gridSize)
            continue; 
        end
        
        
        if readDistance(proximity) < 0.15 
            obstacles(end+1,:) = neighbor;
            continue;
        end
        
        if ismember(neighbor, obstacles, 'rows')
            continue; 
        end
        
        tentative_gScore = gScore(key(current)) + 1;
        neighborKey = key(neighbor);
        
        if ~isKey(gScore, neighborKey) || tentative_gScore < gScore(neighborKey)
            cameFrom(neighborKey) = current;
            gScore(neighborKey) = tentative_gScore;
            fScore(neighborKey) = tentative_gScore + heuristic(neighbor);
            if ~any(cellfun(@(p) isequal(p, neighbor), openList))
                openList{end+1} = neighbor;
            end
        end
    end
end


path = goalPos;
while isKey(cameFrom, key(path(1,:)))
    path = [cameFrom(key(path(1,:))); path];
end


for i = 2:size(path,1)
    moveTo(path(i,:), path(i-1,:), leftMotor, rightMotor, gyro);
end


stop(leftMotor);
stop(rightMotor);