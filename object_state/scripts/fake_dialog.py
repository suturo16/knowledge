#!/usr/bin/env python  
import roslib
roslib.load_manifest('object_state')

import rospy
import tf
import thread
from json_prolog import json_prolog

if __name__ == '__main__':
    #init rosnode, prolog and tflistener
    rospy.init_node('fake_dialog')

    prolog = json_prolog.Prolog()

    # Create the objects

    query = prolog.query("assert_dialog_element('{guestId:michael1,query:{type:setCake,amount:5,guestName:michael}}')")
    for solution in query.solutions():
        print('fake set cake sended')
    query.finish()
    print('----------------------------------------------------------')

    query = prolog.query("assert_dialog_element('{guestId:michael1,query:{type:setLocation,tableId:table1}}')")
    for solution in query.solutions():
        print('fake set location sended')
    query.finish()
    print('----------------------------------------------------------')

    # Get their respective infos
    query = prolog.query("get_open_orders_with_customer_infos(CustomerID,Name,Place,Item,TotalAmount,DeliveredAmount)")
    for solution in query.solutions():
        print(str(solution))
    query.finish()
