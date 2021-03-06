/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *PRIORITY_TCAMERAPRIORITY_TCAMERA
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 25
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 25
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 25
#define PRIORITY_TWATCHROBOT 25
#define PRIORITY_TCHECKBATTERY 19
#define PRIORITY_TCAMERA 22
#define PRIORITY_TPICTURES 19




/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_WD, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_comRobot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_WatchRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_checkBattery, "th_checkBattery", 0, PRIORITY_TCHECKBATTERY,0)){
        cerr <<"Error task create: " << strerror(-err) <<endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_watchrobot, "th_watchrobot", 0, PRIORITY_TWATCHROBOT, 0)) {
        cerr <<"Error task create: " << strerror(-err) <<endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_checkBattery, (void(*)(void*)) & Tasks::CheckBatteryTask, this)) {
        cerr <<"Error task start: " << strerror (-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_watchrobot, (void(*)(void*)) & Tasks::WatchRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
        
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        /* Perte de communication avec le moniteur */
        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            int cp = robotStarted;
            robotStarted=0;
            rt_mutex_release(&mutex_robotStarted);
            
            cout << "Communication with the monitor is lost" <<endl;
            
            //On arrête le robot
            if (cp==1) {
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                Message *msgSend = robot.Write(robot.Stop());
                robot.Close();
                rt_mutex_release(&mutex_robot);
            }
            
            //On ferme le serveur
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Close();
            rt_mutex_release(&mutex_monitor);
            
            break;
        }
        /* Fermeture de la communication entre le robot et le moniteur */
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_CLOSE)){
            
            com = 0;
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            int cp = robotStarted;
            robotStarted=0;
            rt_mutex_release(&mutex_robotStarted);
            
            cout << "Communication with the robot is lost" <<endl;

            if (cp==1) {
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                Message *msgSend = robot.Write(robot.Stop());
                rt_mutex_release(&mutex_robot);

            }
       
        }
        /* Ouverture de la communication entre le robot et le moniteur */
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            com = 1 ;
            rt_sem_v(&sem_openComRobot);
        } 
        /* Démarrage sans WatchDog */
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            start = msgRcv-> GetID();
            rt_sem_v(&sem_startRobot);
        } 
        /* Démarrage avec WatchDog */ 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            start = msgRcv-> GetID();
            rt_sem_v(&sem_startRobot); 
        } 
        /* Déplacement manuel du robot */
        else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
            msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
            msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
            msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
            msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } 
        delete(msgRcv); // must be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) { //status = 0 => Echec de l'ouverture, et donc non-acquittement
            msgSend = new Message(MESSAGE_ANSWER_NACK); 
        } else { //status = 1 => Ouverture réussite, et donc acquittement
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
       
        
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        rt_sem_v(&sem_WatchRobot);
    }
}
 


/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        
        rt_mutex_acquire(&mutex_WD, TM_INFINITE);
        int wd = WD;
        rt_mutex_release(&mutex_WD);
        
        
        
        /* Démarrage du robot sans WatchDog */
        if (wd==0) {
                        
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
            cout << "Movement answer: " << msgSend->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
            
            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            }  
        } 
        /* Démarrage du robot avec WatchDog */
        else if (wd ==1) {
            
            
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
            
            cout << "Movement answer: " << msgSend->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
            
            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            } 
            
            rt_task_set_periodic(NULL, TM_NOW, 1000000000);
            while(robotStarted){
            rt_task_wait_period(NULL);
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.ReloadWD());
            cout <<"RELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOAD\n"<<flush;
            rt_mutex_release(&mutex_robot);
            
            }
        }
    }
}




/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        Message *msgSend;
        rt_task_wait_period(NULL);
        cout << "Periodic movement update ";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1 && com == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
            
            rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                cpt=0;
            } else {
                cout << "Timeout msg" << endl ;
                cpt++;        
            }
            rt_mutex_release(&mutex_comRobot);
        }
        cout << endl << flush;
    }
}

void Tasks::CheckBatteryTask(void * arg){
    int rs;
    Message *msgSend; 
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs == 1) {
            cout << endl << "Battery ... ";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);
            rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
            if (msgSend ->GetID() == MESSAGE_ANSWER_ACK) {
                cpt=0;
            }else {
                cout << "Timeout for battery" << endl ;
                cpt++;
            }
            rt_mutex_release(&mutex_comRobot);
            
            WriteInQueue (&q_messageToMon, msgSend);
            
            cout << "done." << endl << flush ;
        }
        else {
            cout << "Robot stopped." << endl << flush ;
        }
    }
}


void Tasks::WatchRobotTask(void * arg) {
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    
    while(1){
        rt_task_wait_period(NULL);
        Message * msgSend;
        rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
   
        if(cpt > 3) {
            cout << "Connection Robot Lost" << endl;
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            rt_mutex_acquire(&mutex_WD, TM_INFINITE);
            WD = 0;
            rt_mutex_release(&mutex_WD);
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Close();
            rt_mutex_release(&mutex_robot);
            rt_mutex_acquire(&mutex_comRobot, TM_INFINITE);
            cpt = 0;
            rt_mutex_release(&mutex_comRobot);
            msgSend = new Message(MESSAGE_ANSWER_COM_ERROR);
            WriteInQueue(&q_messageToMon, msgSend);
        }
        rt_mutex_release(&mutex_comRobot);
    }
}


/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

