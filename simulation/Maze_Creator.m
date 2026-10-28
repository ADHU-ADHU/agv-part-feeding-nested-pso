%Matrix of the map
% map=[...
%      0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
%     0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
%     0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0;
%     0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
%     0,0,0,0,0,0,1,0,0,1,1,0,0,1,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];

map=[...
     0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
    0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
    0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0;
    0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0;
    0,0,0,0,0,0,1,0,0,1,1,0,0,1,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
%Each 1 coresponds to a cube of 1m^3



% Bring up template and create the obstacles
myworld = vrworld('ScenarioTemplate.x3d');
% open the virtual world
open(myworld);
% create a handle to a node 'AUTOTerrain', the node that will contain the DEM data
Walls_node = vrnode(myworld,'Walls');
%delete(vrnode(myworld,'Walls','children'));
for z=1:20
    for x=1:20
        if(map(z,x)>0.5)
            % create a child of 'Walls' - transform
            newTransform = vrnode(Walls_node, 'children', '', 'Transform');
            newTransform.translation=[-10+(x-0.505)*1,0.5,-10+(z-0.505)*1];
            newShape= vrnode(newTransform, 'children', '', 'Shape');
            % create appearance field for the shape
            newAppear = vrnode(newShape, 'appearance', '', 'Appearance');
            % create material field for the appearance
            newMat = vrnode(newAppear, 'material', '','Material');

            newBox = vrnode(newShape, 'geometry', '','Box');
            % assign properties for the geometry field - use DEM data
            newBox.size=[1.01 1 1.01];

            newTexture = vrnode(newAppear, 'texture', 'Brick_small','ImageTexture');
            newTexture.url = 'texture/Brick_2.jpg';
        end
    end
end

%Save the changes
save(myworld, 'Scenario.x3d')

close(myworld);
pause(2);
%delete(myworld);