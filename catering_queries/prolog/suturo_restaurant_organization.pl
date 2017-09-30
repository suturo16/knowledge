/** <module> suturo_restaurant_organization
rdf_assert_with_literal(S,P,Value)

  Copyright (C) 2017 Sascha Jongebloed
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(suturo_restaurant_organization,
    [
      get_customer_infos/3,
      get_open_orders_of/4,
      get_open_orders_with_customer_infos/6,
      get_free_table/1,
      set_delivered_amount/2,
      increase_delivered_amount/2,
      increase_delivered_amount/1
    ]).

:- rdf_meta get_customer_infos(?,?,r),
      get_open_orders_of(?,?,?,?),
      get_open_orders_with_customer_infos(?,?,r,?,?,?),
      get_free_table(?),
      set_delivered_amount(?,?),
      increase_delivered_amount(?,?),
      increase_delivered_amount(?).


% get_customer_infos(+CustomerID,-Name,-Place) 
%
% Returns the open orders of a customer, with his place
%
% @CustomerID The customer
% @Name Name of the customer
% @Place TableId where the customer is placed
%
%getCustomerInfos(+CustomerID,-Name,-Place)
get_customer_infos(CustomerID,Name,Place) :-
      get_open_orders_with_customer_infos(CustomerID,Name,Place,_,_,_) .

%get_open_orders_of(+CustomerID,-Item,-TotalAmount,-DeliveredAmount)
%
% Returns the open orders of a customer, with his place
%
% @CustomerID The customer
% @Item Ordered products
% @TotalAmount Ordered amount
% @DeliveredAmount The deliveredAmount
%
get_open_orders_of(CustomerID,Item,TotalAmount,DeliveredAmount) :-
      get_open_orders_with_customer_infos(CustomerID,_,_,Item,TotalAmount,DeliveredAmount).

%getOpenOrdersWithCustomerInfos(+CustomerID,-Name,-Place,-Item,-TotalAmount, -DeliveredAmount)
%
% Returns the open orders of a customer, with his place
%
% @CustomerID The customer
% @Name Name of the customer
% @Place TableId where the customer is placed
% @Item Ordered products
% @TotalAmount Ordered amount
% @DeliveredAmount The deliveredAmount
%
get_open_orders_with_customer_infos(CustomerID,Name,Place,Item,TotalAmount,DeliveredAmount) :-
    rdf_has(Obj,knowrob:'guestId',literal(type(xsd:string,CustomerID))), %# rdf_has because holds gives redundant results
    rdf_has(Obj,rdf:type,knowrob:'Customer'), %# rdf_has because holds gives redundant results
    holds(Obj,knowrob:'guestName',literal(type(xsd:string,Name))),
    ignore(holds(Obj,knowrob:'guestName',literal(type(xsd:string,Name)))),
    holds(Obj,knowrob:'visit',Visit),
    ignore((holds(Visit,knowrob:'locatedAt',PlaceU),
      (atom_concat('http://knowrob.org/kb/knowrob.owl#', Place, PlaceU) ->atom_concat('http://knowrob.org/kb/knowrob.owl#', Place, PlaceU) ;Place = PlaceU))),
    holds(Visit,knowrob:'hasOrder',Order),
    holds(Order,knowrob:'itemName',literal(type(xsd:string,Item))),
    holds(Order,knowrob:'orderedAmount',literal(type(xsd:integer,TotalAmountAtom))),
    (atom(TotalAmountAtom) -> atom_number(TotalAmountAtom,TotalAmount) ; TotalAmount = TotalAmountAtom),
    holds(Order,knowrob:'deliveredAmount',literal(type(xsd:integer,DeliveredAmountAtom))), %# This parameter isn#t always needed
    (atom(DeliveredAmountAtom) -> atom_number(DeliveredAmountAtom,DeliveredAmount) ; DeliveredAmount = DeliveredAmountAtom).

%% get_free_table(?NameOfFreeTable) 
%
% Returns free tables in the kitchen
%
% @NameOfFreeTable tableId of free table
%
get_free_table(NameOfFreeTable) :-
      owl_has(Table,rdf:type,knowrob:'RestaurantTable'),
      owl_has(Table,knowrob:'tableId',literal(type(xsd:string,NameOfFreeTable))),
      \+ (holds(_,knowrob:'locatedAt',Table)).

% set_delivered_amount(+CustomerID,+Amount)
%
% Sets the delivered amount for the customer
%
% @CustomerID The customer
% @Amount Amount to set to
%
set_delivered_amount(CustomerID,Amount) :-
    (holds(Obj,knowrob:'guestId',literal(type(xsd:string,CustomerID))),
    holds(Obj,rdf:type,knowrob:'Customer'),!),
    holds(Obj,knowrob:'visit',Visit),
    holds(Visit,knowrob:'hasOrder',Order),
    %# Replace the deliveredAmount value by the new value
    current_time(Now),
    forall((holds(Order,knowrob:'deliveredAmount',O)), 
    assert_temporal_part_end(Order, knowrob:'deliveredAmount', O, Now)),
    assert_temporal_part(Order, knowrob:'deliveredAmount', Amount).

%% increase_delivered_amount(+CustomerID,+Amount)
%
% Increase the delivered amount for the customer
%
% @CustomerID The customer
% @Amount Amount to increase by
%
increase_delivered_amount(CustomerID,Amount) :-
    (holds(Obj,knowrob:'guestId',literal(type(xsd:string,CustomerID))),
    holds(Obj,rdf:type,knowrob:'Customer'),!),
    holds(Obj,knowrob:'visit',Visit),
    holds(Visit,knowrob:'hasOrder',Order),
    holds(Order,knowrob:'deliveredAmount',literal(type(xsd:integer,DeliveredAmount))),
    (atom(DeliveredAmount) -> atom_number(DeliveredAmount,DAAsNumber);DAAsNumber = DeliveredAmount),
    (atom(Amount) -> atom_number(Amount,AmountAsNumber);AmountAsNumber = Amount),
    DeliveredAmountNew is DAAsNumber + AmountAsNumber,
    set_delivered_amount(CustomerID,DeliveredAmountNew).

%% increase_delivered_amount(+CustomerID)
%
% Increase the delivered amount for the customer by 1
%
% @CustomerID The customer
%
increase_delivered_amount(CustomerID) :-
    increase_delivered_amount(CustomerID,1).


