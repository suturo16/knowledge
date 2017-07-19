%%
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
:- register_ros_package(suturo_capabilities).
:- register_ros_package(catering_queries).


:- use_module(library('prolog_object_state')).
:- use_module(library('test_calls')).
:- use_module(library('suturo_capabilities')).
:- use_module(library('srdl2')).
:- use_module(library('knowrob_owl')).
:- use_module(library('swrl')).
:- use_module(library('owl_computable')). % needed for computables in restricted actions
:- use_module(library('catering_queries')).
:- use_module(library('suturo_restaurant_organization')).
:- use_module(library('dialog_management')).
%% further will be added


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_common/owl/knowrob_common.owl').
:- owl_parse('package://object_state/owl/suturo_objects.owl').
:- owl_parse('package://object_state/owl/suturo_actions.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').
:- owl_parse('package://suturo_capabilities/owl/pepper.owl').
:- owl_parse('package://suturo_capabilities/owl/PR2.owl').
:- owl_parse('package://suturo_capabilities/owl/turtlebot.owl').
:- owl_parse('package://suturo_capabilities/owl/suturo-cap.owl').
:- owl_parse('package://catering_queries/owl/suturo_recipes.owl').
:- owl_parse('package://catering_queries/owl/suturo_dialog.owl').