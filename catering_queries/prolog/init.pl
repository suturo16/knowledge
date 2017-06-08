%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(object_state).
:- register_ros_package(catering_queries).

:- use_module(library('catering_queries')).

%% further will be added


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://object_state/owl/suturo_objects.owl').
:- use_module(library('knowrob_owl')).
