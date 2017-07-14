/** <module> suturo_restaurant_organization

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(suturo_restaurant_organization,
    [
      get_customer_infos/3,
      get_open_orders_of/3,
      get_open_orders_with_customer_infos/5,
      get_tree_table/1      
    ]).

%getCustomerInfos(+CustomerID,-Name,-Place)
get_customer_infos(CustomerID,Name,Place) :-
      get_open_orders_with_customer_infos(CustomerID,Name,Place,_,_) .


%getOpenOrdersOf(+customerID,-Item,-Amount)
get_open_orders_of(CustomerID,Item,Amount) :-
      get_open_orders_with_customer_infos(CustomerID,_,_,Item,Amount).


%getOpenOrdersWithCustomerInfos(+CustomerID,-Name,-Place,-Good,-Amount)
get_open_orders_with_customer_infos(CustomerID,Name,Place,Item,Amount) :-
      Name='michael',
      Place='table1',
      Item='cake',
      Amount=1.

%getFreeTables(-NameOfFreeTable)
get_tree_table(NameOfFreeTable) :-
      NameOfFreeTables='table2'.