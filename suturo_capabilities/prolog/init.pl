%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_srdl).
:- register_ros_package(suturo_capabilities).

:- use_module(library('suturo_capabilities')).
:- use_module(library('srdl2')).
:- use_module(library('knowrob_owl')).

%% further will be added
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_common/owl/knowrob_common.owl').

:- owl_parse('package://object_state/owl/suturo_objects.owl').
:- owl_parse('package://suturo_capabilities/owl/pepper.owl').
:- owl_parse('package://suturo_capabilities/owl/PR2.owl').
:- owl_parse('package://suturo_capabilities/owl/turtlebot.owl').
:- owl_parse('package://suturo_capabilities/owl/suturo-cap.owl').
:- owl_parse('package://suturo_capabilities/owl/suturo_actions.owl').


