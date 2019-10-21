%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is part of the hoverboard-new-firmware-hack-FOC project
%
% Author: Emanuel FERU
% Copyright © 2019 Emanuel FERU <aerdronix@gmail.com>
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% =========================================================================
% get_param(gcb, 'BlockType')

% Compile model
BLDCmotorControl_FOC_R2017b_fixdt([],[],[],'compile');
modelName = 'BLDCmotorControl_FOC_R2017b_fixdt';

Parameter   = '';
DataType    = '';

% Search for constants
Blocks = find_system(modelName, 'regexp', 'on', 'BlockType', 'Constant');
for k = 1:length(Blocks)
    
    if strcmp('yellow',get_param(Blocks{k},'BackgroundColor'))  % get only the tunnable parameters
        
        val = get_param(Blocks{k}, 'Value'); % BreakpointsData, Table
        typ = get_param(Blocks{k}, 'CompiledPortDataTypes');
        Parameter{end+1,1}    = val;
        DataType{end+1,1}     = typ.Outport{1};
        
    end
end

% Search for PreLookup
Blocks = find_system(modelName, 'regexp', 'on', 'BlockType', 'PreLookup');
for k = 1:length(Blocks)
    
    if strcmp('yellow',get_param(Blocks{k},'BackgroundColor'))  % get only the tunnable parameters
        
        val = get_param(Blocks{k}, 'BreakpointsData'); % BreakpointsData, Table
        typ = get_param(Blocks{k}, 'CompiledPortDataTypes');
        Parameter{end+1,1}    = val;
        DataType{end+1,1}     = typ.Inport{1};
        
    end
end

% Search for Interpolation_n-D
Blocks = find_system(modelName, 'regexp', 'on', 'BlockType', 'Interpolation_n-D');
for k = 1:length(Blocks)
    
    if strcmp('yellow',get_param(Blocks{k},'BackgroundColor'))  % get only the tunnable parameters
        
        val = get_param(Blocks{k}, 'Table'); % BreakpointsData, Table
        typ = get_param(Blocks{k}, 'CompiledPortDataTypes');
        Parameter{end+1,1}    = val;
        DataType{end+1,1}     = typ.Outport{1};
        
    end
end

% Search for Relay
Blocks = find_system(modelName, 'regexp', 'on', 'BlockType', 'Relay');
for k = 1:length(Blocks)
    
    if strcmp('yellow',get_param(Blocks{k},'BackgroundColor'))  % get only the tunnable parameters
        
        val1 = get_param(Blocks{k}, 'OnSwitchValue');
        val2 = get_param(Blocks{k}, 'OffSwitchValue');
        typ = get_param(Blocks{k}, 'CompiledPortDataTypes');
        Parameter{end+1,1}    = val1;
        Parameter{end+1,1}    = val2;
        DataType{end+1,1}     = typ.Inport{1};
        DataType{end+1,1}     = typ.Inport{1};
        
    end
end

% Terminate compilation
BLDCmotorControl_FOC_R2017b_fixdt([],[],[],'term');

% Create table
T = table(Parameter,DataType);
T = unique(T, 'rows');                  % remove duplicates
T = sortrows(T);                        % sort
writetable(T,'tableParamType.xlsx');    % write to file

disp('---- Parameters Table: Successful ----');