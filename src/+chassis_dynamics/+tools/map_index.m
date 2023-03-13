function iRow = map_index(map_in,map_ref)
%MAP_INDEX Find the index of a given ID,DOF pair

    [~,iRow] = ismember(map_in,map_ref,'rows');
    if iRow == 0
        error('Index was not found');
    end
    
end

