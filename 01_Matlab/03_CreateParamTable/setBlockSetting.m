function setBlockSetting(topLevelModel,block,setting,value)
% This function changes a certain setting for a specific type of blocks found within the main model
% ============================================================
% Example
% setBlockSetting(bdroot, 'Inport', 'BackgroundColor', 'cyan');
% setBlockSetting(bdroot, 'Outport', 'BackgroundColor', 'orange');
% ============================================================

% Load Top Model
if( ~bdIsLoaded( topLevelModel ) )
    load_system( topLevelModel );   % If Model is not already loaded, load it
end


%% Find Model Reference Blocks in the top level model:
topLevelModelHandle = get_param( topLevelModel , 'Handle' );
blkHandles = find_system( topLevelModelHandle, 'findall', 'on', 'FollowLinks', 'off', 'LookUnderMasks', 'on', 'blocktype', block); 

if( ~isempty( blkHandles ) )   % If the model contains model references    
    
    for k = 1 : length(blkHandles)
        set_param(blkHandles(k), setting, value);
    end
       
end

disp('---- Setting done ----');