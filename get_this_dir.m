function root_dir = get_this_dir(varargin)
    [root_dir,~,~] = fileparts(mfilename('fullpath'));

    if nargin == 0
        return;
    else
        dir_name = varargin{1};
        full_name = fullfile(root_dir, dir_name);
        if ~exist(full_name)
            mkdir(full_name);
        end
        root_dir = full_name;
        return
    end
    
end