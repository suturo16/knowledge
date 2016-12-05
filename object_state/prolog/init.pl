%%
%% Copyright (C) 2010 by Moritz Tenorth
%% Modified by Sascha Jongebloed
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(object_state).

:- use_module(library('prolog_object_state')).
%% further will be added


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

%%:- owl_parse('package://data_model/owl/training.owl').
:- rdf_db:rdf_register_ns(knowrob_perception_tutorial, 'http://knowrob.org/kb/knowrob_perception_tutorial.owl#', [keep(true)]).
