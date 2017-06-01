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
:- register_ros_package(knowrob_actions).
:- register_ros_package(object_state).
:- register_ros_package(knowrob_srdl).


:- use_module(library('prolog_object_state')).
:- use_module(library('srdl2')).
:- use_module(library('knowrob_owl')).
:- use_module(library('swrl')).
:- use_module(library('owl_computable')). % needed for computables in restricted actions
%% further will be added


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://object_state/owl/suturo_objects.owl').
:- owl_parse('package://object_state/owl/suturo_actions.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').
:- owl_parse('package://suturo_cap/owl/pepper.owl').
:- owl_parse('package://suturo_cap/owl/PR2.owl').
