/** <module> prolog_object_state

@author Lukas Samel, Michael Speer, Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(prolog_object_state,
    [
      create_fluent_pose/2,
      create_object_state/9,
      create_object_state/10,
      create_object_state_with_close/9,
      create_object_state_with_close/10,
      create_object_name/2,
      create_temporal_name/2,
      assign_obj_class/2,
      close_object_state/1,
      connect_frames/2,
      disconnect_frames/2,
      isConnected/2,
      multiple_objects_name/2,
      get_class_name/2,
      get_fluent_pose/3,
      get_info/2,
      get_info/3,
      get_object/2,
      get_object_infos/5,
      get_object_infos/6,
      get_object_infos/8,
      get_object_infos/9,
      get_object_infos_to_odom/5,
      get_object_infos_to_odom/6,
      get_physical_parts/3,
      get_tf_infos/4,
      get_max_num/2,
      get_type_num/2,
      get_current_temporal_part_time/2,
      known_object/6,
      same_dimensions/2,
      same_position/3,
      seen_since/3,
      set_info/2
    ]).

:- dynamic
        isConnected/2.

:- rdf_meta create_object_state(r,r,r,r,r,r,r,r,?),
      close_object_state(r,r,r,r,r,r,r,r,?),
      create_object_state_with_close(r,r,r,r,r,r,r,r,?),
      create_object_name(r,?),
      create_temporal_name(r,?),
      create_physical_parts(r,r),
      set_info(r,r),
      set_info(r,r,r),
      get_info(r,r),
      get_info(r,r,?),
      get_object_infos(r,?,?,?,?),
      get_object_infos(r,?,?,?,?,?),
      get_object_infos(?,?,?,?,?,?,?,?),
      connect_frames(r,r),
      get_type_num(r,?),
      disconnect_frames(r,r),
      seen_since(r,r,r).

%importing external libraries
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('knowrob_temporal')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_owl')).
:- use_module(library('srdl2')).
:- use_module(library('rdfs_computable')).
:- use_module(library('prython')).
:- use_module(library('swrl')).


%registering namespace
%#:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_obj, 'package://object_state/owl/suturo_object.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_act, 'package://object_state/owl/suturo_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_cap, 'http://knowrob.org/kb/suturo-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pepper, 'http://knowrob.org/kb/pepper.owl', [keep(true)]).
:- rdf_db:rdf_register_ns(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_swrl, 'http://knowrob.org/kb/swrl_test#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).

%parse libraries
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').
:- owl_parse('package://knowrob_common/owl/swrl_test.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').


% Initialize python context
:- source_file(File),
   string_concat(Path,'/prolog_object_state.pl',File),
   string_concat(Path,'/../scripts',FullPath),
   add_py_path(FullPath).
 
%% create_object_state(+Name, +Pose, +Type, +FrameID, +Width, +Height, +Depth, +Begin, -ObjInst) is probably det.
% Create the object representations in the knowledge base
% Argument 'Type' specifies perceptions classification of the object
% with multiple objects the Type from perception is droped and replaced with the name, e.g. Type = cake
% 
% @param Name describes the class of the object
% @param PoseAsList The pose of the object stored in a list of lists
% @param Type The type of the object (see ObjectDetection.msg)
% @param FrameID reference frame of object
% @param Width The width of the object
% @param Height The height of the object
% @param Depth The depth of the object
% @param Interval A list containing the start time and end time of a temporal
% @param ObjInst The created object instance (optional:to be returned)
create_object_state(Name, Pose, Type, FrameID, Width, Height, Depth, [Begin], ObjInst) :- 
    (nonvar(Name)
    -> holds(ObjInst,knowrob:'nameOfObject',Name)
      ; assign_obj_class(Type,ObjInst)),
    
    %previously used was this:
    %create_fluent(ObjInst, Fluent), fluent_assert)(S,P,O)
    assert_temporal_part(ObjInst, knowrob:'typeOfObject', Type),      % literal(type(xsd:string, Type))),
    assert_temporal_part(ObjInst, knowrob:'frameOfObject', FrameID),  % literal(type(xsd:string, FrameID))),
    assert_temporal_part(ObjInst, knowrob:'widthOfObject', Width),    % literal(type(xsd:float, Width))),
    assert_temporal_part(ObjInst, knowrob:'heightOfObject', Height),  % literal(type(xsd:float, Height))),
    assert_temporal_part(ObjInst, knowrob:'depthOfObject', Depth),    % literal(type(xsd:float, Depth))),
    create_fluent_pose(ObjInst, Pose),
    create_fluent_pose_to_odom(ObjInst, Pose),
    (holds(ObjInst,knowrob:'physicalParts',_) ->
    	true;
    	ignore(create_physical_parts(Type,ObjInst))).


create_object_state(Name, Pose, PoseToOdom, Type, FrameID, Width, Height, Depth, [Begin], ObjInst) :- 
    (nonvar(Name)
    -> holds(ObjInst,knowrob:'nameOfObject',Name)
      ; assign_obj_class(Type,ObjInst)),
    assert_temporal_part(ObjInst, knowrob:'typeOfObject', Type),      % literal(type(xsd:string, Type))),
    assert_temporal_part(ObjInst, knowrob:'frameOfObject', FrameID),  % literal(type(xsd:string, FrameID))),
    assert_temporal_part(ObjInst, knowrob:'widthOfObject', Width),    % literal(type(xsd:float, Width))),
    assert_temporal_part(ObjInst, knowrob:'heightOfObject', Height),  % literal(type(xsd:float, Height))),
    assert_temporal_part(ObjInst, knowrob:'depthOfObject', Depth),    % literal(type(xsd:float, Depth))),
    create_fluent_pose(ObjInst, Pose),
    create_fluent_pose_to_odom(ObjInst, PoseToOdom).


%% create_object_state_with_close(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, (+)[Begin], -ObjInst)
% LSa, MSp
% Creates a fluent and closes the corresponding old TemporalPart.
create_object_state_with_close(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    known_object(Type, Pose, Width, Height, Depth, FullName),!
        -> (atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
          atom_concat('/', Name, ChildFrameID),
          not(isConnected(_ ,ChildFrameID))
            -> ignore(close_object_state(FullName)),
            create_object_state(FullName, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst)
            ; false)
        ; create_object_state(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst).


%% create_object_state_with_close(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, (+)[Begin], -ObjInst)
% LSa, MSp
% Creates a fluent and closes the corresponding old TemporalPart.
create_object_state_with_close(_, Pose, PoseToOdom, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    known_object(Type, PoseToOdom, Width, Height, Depth, FullName)
      -> (atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
         atom_concat('/', Name, ChildFrameID),
          not(isConnected(_ ,ChildFrameID))
            -> ignore(close_object_state(FullName)),
            create_object_state(FullName, Pose, PoseToOdom, Type, Frame, Width, Height, Depth, [Begin], ObjInst)
            ; false)
		; create_object_state(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst).


%% assign_obj_class(+Type, -ObjInst)
% MSp
% Initialized object of class from suturo_objects.owl ontology depending on object type.
assign_obj_class(Type, ObjInst) :-
    Ns = knowrob,
    get_class_name(Type, Name),
    rdf_global_id(Ns:Name, Id),
    owl_subclass_of(Id, Class),
    rdf_instance_from_class(Class, ObjInst),
    % #Creates name
    multiple_objects_name(Type, NameNum), 
    create_object_name(NameNum, FullName),
    rdf_assert(ObjInst, knowrob:'nameOfObject',FullName),
    %# Adds the physicalParts. BE CAREFUL: nameOfObject for ObjInst needed
    ignore(create_physical_parts(Type,ObjInst)),!.


%Creates individuals for all physical parts
create_physical_parts(Type,ObjInst) :-
    % build knowrob:Class
    Ns = knowrob,
    get_class_name(Type, Name),
    rdf_global_id(Ns:Name, Id),
    % build parent frame id
    owl_has(ObjInst,knowrob:'nameOfObject',ParentName),
    create_object_name(ParentNameWithoutKnowrob, ParentName),
    atom_concat('/', ParentNameWithoutKnowrob, ParentFrameID),
    % foreach physical part assert connection to object
    forall((
      rdf_has(Id,rdfs:subClassOf,A),
      rdf_has(A,owl:onProperty,knowrob:'physicalParts'),
      rdf_has(A,owl:onClass,PartClass)
    ),(
      rdf_instance_from_class(PartClass,PartInd),
      rdf_assert(ObjInst,knowrob:'physicalParts',PartInd),
      create_object_name(PhysicalPartType, PartClass),
      multiple_objects_name(PhysicalPartType, NameNum), 
      create_object_name(NameNum, FullName),
      rdf_assert(PartInd,knowrob:'nameOfObject',FullName),
      rdf_assert(PartInd,knowrob:'frameOfObject',ParentFrameID))
    ).


%% get_class_name(+Type, -ClassName)
% MSp
% converts first letter of Type into capital letter
get_class_name(Type, ClassName) :-
    not((var(Type), var(ClassName))),
    sub_atom(Type,0,1,_,C), char_code(C,I), 96<I, I<123
    -> J is I-32, char_code(D,J), sub_atom(Type,1,_,0,Sub), atom_concat(D,Sub,ClassName)
    ; ClassName = Type.


%% create_fluent_pose(+Fluent, +Pose)
% MSp
% @param Fluent temporal part of object
% @param Pose list of lists [[3],[4]] position and orientation
create_fluent_pose(ObjInst, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
	PXVal_=literal(type(xsd:'float',PX)), rdf_global_term(PXVal_, PXVal),
    assert_temporal_part(ObjInst, knowrob:'xCoord', PXVal), % literal(type(xsd:float, PX))),
	PYVal_=literal(type(xsd:'float',PY)), rdf_global_term(PYVal_, PYVal),
    assert_temporal_part(ObjInst, knowrob:'yCoord', PYVal), % literal(type(xsd:float, PY))),
	PZVal_=literal(type(xsd:'float',PZ)), rdf_global_term(PZVal_, PZVal),
    assert_temporal_part(ObjInst, knowrob:'zCoord', PZVal), % literal(type(xsd:float, PZ))),
	OXVal_=literal(type(xsd:'float',OX)), rdf_global_term(OXVal_, OXVal),
    assert_temporal_part(ObjInst, knowrob:'qx', OXVal), % literal(type(xsd:float, OX))),
	OYVal_=literal(type(xsd:'float',OY)), rdf_global_term(OYVal_, OYVal),
    assert_temporal_part(ObjInst, knowrob:'qy', OYVal), % literal(type(xsd:float, OY))),
	OZVal_=literal(type(xsd:'float',OZ)), rdf_global_term(OZVal_, OZVal),
    assert_temporal_part(ObjInst, knowrob:'qz', OZVal), % literal(type(xsd:float, OZ))),
	OUVal_=literal(type(xsd:'float',OW)), rdf_global_term(OUVal_, OUVal),
    assert_temporal_part(ObjInst, knowrob:'qu', OUVal). % literal(type(xsd:float, OW))).


%% create_fluent_pose_to_odom(+Fluent, +Pose)
% MSp
% @param Fluent temporal part of object
% @param Pose list of lists [[3],[4]] position and orientation
create_fluent_pose_to_odom(ObjInst, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
    PXVal_=literal(type(xsd:'float',PX)), rdf_global_term(PXVal_, PXVal),
    assert_temporal_part(ObjInst, knowrob:'xCoordToOdom', PXVal), % literal(type(xsd:float, PX))),
	PYVal_=literal(type(xsd:'float',PY)), rdf_global_term(PYVal_, PYVal),
    assert_temporal_part(ObjInst, knowrob:'yCoordToOdom', PYVal), % literal(type(xsd:float, PY))),
	PZVal_=literal(type(xsd:'float',PZ)), rdf_global_term(PZVal_, PZVal),
    assert_temporal_part(ObjInst, knowrob:'zCoordToOdom', PZVal), % literal(type(xsd:float, PZ))),
	OXVal_=literal(type(xsd:'float',OX)), rdf_global_term(OXVal_, OXVal),
    assert_temporal_part(ObjInst, knowrob:'qxToOdom', OXVal), % literal(type(xsd:float, OX))),
	OYVal_=literal(type(xsd:'float',OY)), rdf_global_term(OYVal_, OYVal),
    assert_temporal_part(ObjInst, knowrob:'qyToOdom', OYVal), % literal(type(xsd:float, OY))),
	OZVal_=literal(type(xsd:'float',OZ)), rdf_global_term(OZVal_, OZVal),
    assert_temporal_part(ObjInst, knowrob:'qzToOdom', OZVal), % literal(type(xsd:float, OZ))),
	OUVal_=literal(type(xsd:'float',OW)), rdf_global_term(OUVal_, OUVal),
    assert_temporal_part(ObjInst, knowrob:'quToOdom', OUVal). % literal(type(xsd:float, OW))).


%% close_object_state(+FullName) is probably det.
% SJo
% Closes the interval of a holding fluent 
% @param Name describes the class of the object
close_object_state(FullName) :-
    holds(ObjInst, knowrob:'nameOfObject', FullName),
    % FIXME: Should be replaced by fluent_assert_end if it works.
    current_time(Now),
    forall(
    	(holds(ObjInst,P,O),O \= FullName, not(rdf_equal(P,knowrob:physicalParts))), 
    	assert_temporal_part_end(ObjInst, P, O, Now)),!.

%    rdf_has(ObjInst, knowrob:'temporalParts',SubjectPart),
%    rdf_has(SubjectPart, P, _),
%    rdf_has(SubjectPart, knowrob:'temporalExtend', I),
%    not( rdf_has(I, knowrob:'endTime', _) ),
%    create_timepoint(Now, IntervalEnd),
%    rdf_assert(I, knowrob:'endTime', IntervalEnd).
%    fluent_assert_end(Obj,P).
    

%% multiple_objects_name(+Type, -NameNum)
% MSp
% creates object name depending on how many other objects of same type exist
% @param Name type leading to NameNum by appending iterating number
% @param NameNum is the unique name of the new object in KB, e.g. cake32
multiple_objects_name(Type, NameNum) :-
    (get_max_num(Type, Num), number(Num)
    -> Number is Num+1; Number is 1),
    atom_concat(Type, Number, NameNum).


%% get_max_num(+Type, - Number)
% MSp
% gets maximum number from name of object type
get_max_num(Type, Number) :-
    get_type_num(Type, NumA), get_type_num(Type, NumB), NumA > NumB
    -> get_type_num(Type, Number), get_type_num(Type, NumC), Number > NumC;
      get_type_num(Type, Number).


%% get_type_num(+Type, -Number)
% MSp
% helper funciton for to get max Number in NameNum
get_type_num(Type, Number) :-
    get_object_infos(FullName,_,Type,_,_,_,_,_), 
    create_object_name(NameNum,FullName),
    atom_concat(Type, NumChar, NameNum), 
    atom_number(NumChar, Number).


%% strip_name_num(+NameNum, -Name)
% MSp
% removes appended numbers from object name
% @param NameNum is the name with appended type count, e.g. cake32
% @param Name is the type of the object
strip_name_num(NameNum, Name) :-
    sub_atom(NameNum, _,1,0,L), atom_number(L, _)
    -> sub_string(NameNum,0,_,1, Sub), strip_name_num(Sub, Name);
      Name = NameNum.


%% create_object_name(+Name,-FullName) is det.
% LSa
% Appends Name to the knowrob domain.
% @param Name content to be appended to the namespace
% @preturns FullName the concatenated string
create_object_name(Name, FullName) :-
    atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName).


%% create_temporal_name(+FullName, -FullTemporalName) is det.
%
% @param FullName full object name without temporal stamp  perception 
% @returns FullTemporalName the concatenated string including the temporal stamp
create_temporal_name(FullName, FullTemporalName) :-
    atom_concat(FullName,'@t_i', FullTemporalName).


%%##########################################################################################################
%%################################### Ja, Servus und so weiter !!! #########################################
%%##########################################################################################################
%%################################################  NEU  ###################################################
%%######################################## brought to you by me ############################################
%%##########################################################################################################
%%##########################################################################################################
set_info(Thing, Info) :-
    not(rdf_has(Thing, rdf:type, _)),(
    % #if specified by name update object
    (member([nameOfObject,Name],Info),
    holds(ObjInst, knowrob:nameOfObject, Name);
    holds(ObjInst, knowrob:nameOfObject, Thing)),
    set_info(ObjInst, Info);
    % #if Thing is instance of class in knowrob
    owl_subclass_of(Id, knowrob:'SpatialThing'), 
    rdf_global_id(knowrob:Thing,Id), 
    assign_obj_class(Thing, ObjInst),
    set_info(ObjInst, Info)
    % #if Thing is not specified but can be uniquely identified
    % #TODO check and uncomment if required
    % setof(Obj, (setof(X, (member(X, Info), is_list(X)), Conds),
    %            get_object(Obj)), Objs),
    % length(Objs, 1), [ObjInst|_] = Objs,
    % set_info(ObjInst, Info).
    ),!.

%% set_info(ObjInst, [[P,Val]|More])
% MSp
% saves knowledge value O as relation P to S
% @param ObjInst the target object or thing for the P related value Val
% @param P the relation the value Val has to subject S
% @param Val the value of the P related object
set_info(ObjInst, [[P,Val]|More]) :-
    rdf_has(ObjInst, rdf:type, _), % #check that ObjInst is object
    Ns = knowrob,
    rdf_global_id(Ns:P, Id),
    ignore(forall( holds(ObjInst, Id, EndVal),
    assert_temporal_part_end(ObjInst, Id, EndVal))),
    assert_temporal_part(ObjInst, Id, Val),
    set_info(ObjInst, More),!.

set_info(_, []).

%% get_info(+Variables, -Returns)
get_info(Variables, Returns) :-
    is_list(Variables), var(Returns),
    findall(X, (member(X, Variables), is_list(X)), Conds),
    findall(Y, (member(Y, Variables), not(is_list(Y))), Vars),
    get_object(Conds, ObjInst),
    (not(length(Vars,0)), get_info(Vars, Returns, ObjInst);
      length(Vars,0), get_info(ObjInst, Returns)).

get_info(ObjName, Returns) :-
    not(is_list(ObjName)), var(Returns),
    Ns = knowrob, rdf_global_id(Ns:ObjName, ObjInst),
    rdf_has(ObjInst, rdf:type, _),
    get_info(ObjInst, Returns).

get_info(ObjInst, Returns) :-
    not(is_list(ObjInst)), var(Returns), 
    % #check that ObjInst is object:
    rdf_has(ObjInst, rdf:type, _),  
    Ns = knowrob,
    findall([Var,Val], (
      holds(ObjInst, NsVar, RDFvalue),
      rdf_global_id(Ns:Var, NsVar),
      once(
        rdf_global_id(_:Val, RDFvalue); strip_literal_type(RDFvalue, Val))),
      Returns).

%% get_info(Vars, Rets, ObjInst)
% MSp
get_info(Variables,Returns, ObjInst) :-
	not(is_list(ObjInst)),var(Returns),
    Ns = knowrob,
    findall([Var, Val], (
      member(Var, Variables),
      rdf_global_id(Ns:Var, NsVar),
      holds(ObjInst, NsVar, RDFvalue),
      once(
        rdf_global_id(_:Val, RDFvalue); strip_literal_type(RDFvalue, Val))),
      Returns).


%% get_object(+Conditions, -ObjInst)
% MSp
% Helper to identify a certain Object according to given Conditions
get_object([[Cond,Val]|Conds], ObjInst) :-
    Ns = knowrob,
    rdf_global_id(Ns:Cond, NsVar),
    holds(ObjInst, NsVar, RDFvalue), 
    once( 
      rdf_global_id(_:Val, RDFvalue) ; strip_literal_type(RDFvalue, Val)),
    get_object(Conds, ObjInst).

get_object([], _).

%%################################################  Alt  ###################################################

 
get_object_infos(Name, FrameID, Height, Width, Depth) :-
    get_object_infos(Name, FrameID, _, _, _, Height, Width, Depth,_).


get_object_infos(Name, FrameID, Timestamp, Height, Width, Depth) :-
    get_object_infos(Name, FrameID, _, Timestamp, _, Height, Width, Depth,_).


get_object_infos(Name, FrameID, Type, Timestamp, [Position, Orientation], Height, Width, Depth) :-
    get_object_infos(Name, FrameID, Type, Timestamp, [Position, Orientation], Height, Width, Depth,_).


%% get_object_infos(+Name, -FrameID, -Type, -Timestamp, -Pose, -Height, -Width, -Depth, -Obj)
% LSa, MSp
% @param Name name of the object
% @param FrameID reference frame of object pose
% @param Timestamp start time of most recent perception
% @param Type type of the object
% @param Pose list of [Position, Orientation] of object
% @param Height height of object
% @param Width width of object
% @param Depth depth of object
% @param Obj object ID in KB
get_object_infos(Name, FrameID, Type, Timestamp, [Position, Orientation], Height, Width, Depth, Obj) :-
    holds(Obj, knowrob:'typeOfObject', Type),       % literal(type(xsd:string,Type))),
    owl_has(Obj,knowrob:'nameOfObject', Name),
    holds(Obj, knowrob:'frameOfObject', FrameID),   % literal(type(xsd:string,FrameID))),
    holds(Obj, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    holds(Obj, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    holds(Obj, knowrob:'depthOfObject', literal(type(xsd:float,Depth))),
    get_fluent_pose(Obj, Position, Orientation),
   	get_current_temporal_part_time(Obj,Timestamp).


get_physical_parts(Name, PhysicalParts, PhysicalPartName) :-
    (owl_has(Obj,knowrob:'nameOfObject', Name),!),
    holds(Obj,knowrob:'physicalParts',PhysicalParts),
    owl_has(PhysicalParts,knowrob:'nameOfObject', PhysicalPartName).


get_current_temporal_part_time(ObjInst,Timestamp) :-
	temporal_part(ObjInst, TemporalPart, TemporalExtend),
	\+ rdf_has(TemporalExtend, knowrob:endTime, _),
	rdf_has(TemporalExtend,knowrob:startTime,TimePoint),
	create_timepoint(TimestampStr,TimePoint),
	atom_number(TimestampStr,Timestamp),!.

get_object_infos_to_odom(Type, [Position, Orientation], Height, Width, Depth) :-
    holds(Obj, knowrob:'typeOfObject', Type),
    holds(Obj, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    holds(Obj, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    holds(Obj, knowrob:'depthOfObject', literal(type(xsd:float,Depth))),
    get_fluent_pose_to_odom(Obj, Position, Orientation).


get_object_infos_to_odom(Name, Type, [Position, Orientation], Height, Width, Depth) :-
    holds(Obj,knowrob:'nameOfObject',Name),
    holds(Obj, knowrob:'typeOfObject', Type),
    holds(Obj, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    holds(Obj, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    holds(Obj, knowrob:'depthOfObject', literal(type(xsd:float,Depth))),
    get_fluent_pose_to_odom(Obj, Position, Orientation).
    get_current_temporal_part_time(ObjInst,Timestamp).


%% seen_since(+Name, +FrameID, +TimeFloat) --> true/false
%  MSp
%  @param Name name of the object in database
%  @param FrameID reference frame of object
%  @param TimeFloat timestamp to be asserted
seen_since(Name, FrameID, TimeFloat) :-
    get_object_infos(Name, FrameID, _, Timestamp, _, _, _, _),
    TimeFloat < Timestamp;
    close_object_state(Name).


%% get_tf_infos(-Name, -FrameID, -Position, -Orientation)
% LSa
% A function to bundle the required information for the TF-broadcaster.
% @param Name 
% @param FrameID 
% @param Position position of object in frame
% @param Orientation orientation of object in frame
get_tf_infos(Name, FrameID, Position, Orientation) :-
    holds(Obj,knowrob:'nameOfObject',FullName),
    create_object_name(Name, FullName),
    holds(Obj, knowrob:'frameOfObject', FrameID),   % literal(type(xsd:string,FrameID))),
    (holds(Obj, knowrob:'xCoord', _) -> 
      get_fluent_pose(Obj, Position, Orientation); 
      get_pose(Obj, Position, Orientation)).


%% get_fluent_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW])
% MSp
get_fluent_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW]) :-
    holds(Object, knowrob:'xCoord', literal(type(xsd: float, PX))),
    holds(Object, knowrob:'yCoord', literal(type(xsd: float, PY))),
    holds(Object, knowrob:'zCoord', literal(type(xsd: float, PZ))),
    holds(Object, knowrob:'qx', literal(type(xsd: float, OX))),
    holds(Object, knowrob:'qy', literal(type(xsd: float, OY))),
    holds(Object, knowrob:'qz', literal(type(xsd: float, OZ))),
    holds(Object, knowrob:'qu', literal(type(xsd: float, OW))).

%% get_fluent_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW])
% MSp
get_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW]) :-
    owl_has(Object, knowrob:'xCoord', literal(type(xsd: float, PX))),
    owl_has(Object, knowrob:'yCoord', literal(type(xsd: float, PY))),
    owl_has(Object, knowrob:'zCoord', literal(type(xsd: float, PZ))),
    owl_has(Object, knowrob:'qx', literal(type(xsd: float, OX))),
    owl_has(Object, knowrob:'qy', literal(type(xsd: float, OY))),
    owl_has(Object, knowrob:'qz', literal(type(xsd: float, OZ))),
    owl_has(Object, knowrob:'qu', literal(type(xsd: float, OW))).


%% get_fluent_pose_to_odom(Object, [PX, PY, PZ],[OX, OY, OZ, OW])
% MSp
get_fluent_pose_to_odom(Object, [PX, PY, PZ],[OX, OY, OZ, OW]) :-
    holds(Object, knowrob:'xCoordToOdom', literal(type(xsd: float, PX))),
    holds(Object, knowrob:'yCoordToOdom', literal(type(xsd: float, PY))),
    holds(Object, knowrob:'zCoordToOdom', literal(type(xsd: float, PZ))),
    holds(Object, knowrob:'qxToOdom', literal(type(xsd: float, OX))),
    holds(Object, knowrob:'qyToOdom', literal(type(xsd: float, OY))),
    holds(Object, knowrob:'qzToOdom', literal(type(xsd: float, OZ))),
    holds(Object, knowrob:'quToOdom', literal(type(xsd: float, OW))).


%% known_object(+Type, +Pose, +Height, +Width, +Depth, -Name)
%MSp
% same_dimensions currently not used
known_object(Type, [Position, _], Height, Width, Depth, Name) :-
    get_object_infos_to_odom(Name, Type, [PrevPosition, _], PrevHeight, PrevWidth, PrevDepth),
    (%same_dimensions([PrevHeight, PrevWidth, PrevDepth], [Height, Width, Depth]);
	same_position(PrevPosition, Position, [Height, Width, Depth])).


%% same_dimensions(+[PrevDim], +[CurDim])
%MSp
%@param PrevDim dimensions of object at previous timestamp
%@param CurDim  dimensions of object at current  timestamp
same_dimensions([E|PrevDim], CurDim) :-
    length(PrevDim, 0) -> member(E, CurDim);
    same_dimensions([E], CurDim), same_dimensions(PrevDim, CurDim).


%% same_position(+[PrevPos], +[CurPos], +[Dimensions])
%MSp
%@param PrevPos position of object at previous timestamp
%@param CurPos  position of object at current  timestamp
%@param Dimensions tolerance allowed between positions to consider unchanged is maximum dimension of object
same_position(PrevPos, CurPos, Dimensions) :-
    euclidean_dist(PrevPos, CurPos, Dist),
    max_member(Dmax, Dimensions),
    Dist < Dmax.
    

%% euclidean_dist(+[PointA], +[PointB], -Dist)
% MSp
euclidean_dist(PointA, PointB, Dist) :-
    length(PointA, Len), length(PointB, Len) ->
    sqr_sum(PointA, PointB, SqrSum), Dist is sqrt(SqrSum); false.


%% sqr_sum(+[A1|An], +[B1|Bn], -SqrSum)
% MSp
sqr_sum([A1|An], [B1|Bn], SqrSum) :-
    length(An, 0) ->
    SqrSum is ((A1-B1)^2.0);
    sqr_sum(An, Bn, Sum),
    SqrSum is Sum + ((A1-B1)^2.0).

%% connect_frames(+ParentFrameID, +ChildFrameID, +Pose)
% LSa
% A small function to connect two given frames.
connect_frames(ParentFrameID, ChildFrameID) :-
  prython:py_call('call_tf','get_transform',[ParentFrameID,ChildFrameID],Pose),
  write(Pose),
  atom_concat('/', Name, ChildFrameID),
  atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
  get_object_infos_to_odom(FullName, Type, PoseToOdom, Height, Width, Depth),
  create_object_state_with_close(Name, Pose, PoseToOdom, Type, ParentFrameID, Width, Height, Depth, [Begin], ObjInst),
  assert(isConnected(ParentFrameID, ChildFrameID)).


%% disconnect_frames(+ParentFrameID, +ChildFrameID)
% LSa
% A simple function to disconnect two given frames.
disconnect_frames(ParentFrameID, ChildFrameID) :-
  retract(isConnected(ParentFrameID, ChildFrameID)).
