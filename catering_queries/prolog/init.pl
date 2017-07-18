%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(catering_queries).

:- use_module(library('catering_queries')).
:- use_module(library('suturo_restaurant_organization')).
:- use_module(library('dialog_management')).

%% further will be added
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_common/owl/knowrob_common.owl').

:- owl_parse('package://object_state/owl/suturo_objects.owl').
:- owl_parse('package://catering_queries/owl/suturo_recipes.owl').
:- owl_parse('package://catering_queries/owl/suturo_cafetaria.owl').

:- use_module(library('knowrob_owl')).
