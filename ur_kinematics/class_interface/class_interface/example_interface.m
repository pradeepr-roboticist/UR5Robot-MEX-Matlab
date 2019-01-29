%EXAMPLE_INTERFACE Example MATLAB class wrapper to an underlying C++ class
classdef example_interface < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = example_interface(varargin)
            this.objectHandle = example_mex('new', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            example_mex('delete', this.objectHandle);
        end

        %% Train - an example class method call
        function varargout = train(this, varargin)
            [varargout{1:nargout}] = example_mex('train', this.objectHandle, varargin{:});
        end

        %% Test - another example class method call
        function varargout = test(this, varargin)
            [varargout{1:nargout}] = example_mex('test', this.objectHandle, varargin{:});
        end
    end
end