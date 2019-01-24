/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 * 
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima_84@yahoo.com)
 * 
 * Helper object to simulate actions. Query KB for action effects so later those can be applied to
 * KB and update actions (simulate actions).
 * 
 */

#include <ros/ros.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorService.h>
#include <rosplan_knowledge_msgs/DomainOperator.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>

class ActionSimulator
{
    public:
        ActionSimulator();
        ~ActionSimulator();
        
        /**
         * @brief Create service clients for various KB services, store them in member variables for future use
         */
        void prepareServices();
        
        /**
         * @brief Check if service exists within a timeout of 10 secs
         * @param srv any ros::ServiceClient client already initialized
         * @return true if service exists, false otherwise
         */
        bool checkServiceExistance(ros::ServiceClient &srv);
        
        /**
         * @brief returns by reference a list of domain operator names
         * @param operator_names return value will be written here, pass an empty list of strings
         * @return true if KB domain service was available, false otherwise
         */
        bool getOperatorNames(std::vector<std::string> &operator_names);
        
        /**
         * @brief get domain operator details, save them in member variable
         * @return true if KB service exists, false otherwise
         */
        bool saveOperatorNames();
        
        /**
         * @brief get from KB a single domain operator detail, based on input name (operator_name)
         * @param operator_name the name of the operator (action) from which we want to query it's details
         * @param op return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getOperatorDetails(std::string &operator_name, rosplan_knowledge_msgs::DomainOperator &op);
        
        /**
         * @brief fetch some domain operator details (depending on operator_names) and store them locally for future use
         * @param operator_names a list of operators from which we want to query their details
         * @param domain_operator_details return value gets written here by reference
         * @return true if input operator_names size is greater than 0, false otherwise
         */
        bool getSomeOperatorDetails(std::vector<std::string> &operator_names,
            std::vector<rosplan_knowledge_msgs::DomainOperator> &domain_operator_details);
        
        /**
         * @brief fetch all domain operator details and store them locally for future use
         * @param domain_operator_details return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getAllOperatorDetails(std::vector<rosplan_knowledge_msgs::DomainOperator> &domain_operator_details);
        
        /**
         * @brief get all domain operator details and store them in member variable for future use
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool saveAllOperatorDetails();
        
        /**
         * @brief get all domain predicate details
         * @param domain_predicate_details return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getPredicatesDetails(std::vector<rosplan_knowledge_msgs::DomainFormula> &domain_predicate_details);
        
        /**
         * @brief get all domain predicate details and store them in member variable for future use
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool saveAllPredicatesDetails();
        
        /**
         * @brief get all domain predicates details, extract their names, return them by reference
         * @param domain_predicates return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getAllPredicateNames(std::vector<std::string> &domain_predicates);
        
        /**
         * @brief get all domain predicates details, extract their names and store them in member variable list
         * @return this function calls GetAllPredicateNames(), it returns whatever received from it
         */
        bool saveAllPredicateNames();
        
        /**
         * @brief get all operator (action) names, and make a map against their details
         * @param 
         * @return
         */
        bool makeOperatorDetailsMap();
        
        /**
         * @brief get all grounded facts related with a specific predicate_name
         * @param predicate_name the name of the predicate to search in KB
         * @param facts return value gets written here by reference
         * @return this function first calls GetAllPredicateNames(), if successful then will return true
         */
        bool getGroundedPredicates(std::string &predicate_name, std::vector<rosplan_knowledge_msgs::KnowledgeItem> &facts);
        
        /**
         * @brief get all grounded facts in the KB, performs multiple calls to getGroundedPredicates()
         * @param all_facts return value gets written here by reference
         * @return this function performs a call to GetAllPredicateNames(), if successful then will return true
         */
        bool getAllGroundedPredicates(std::vector<rosplan_knowledge_msgs::KnowledgeItem> &all_facts);
        
        /**
         * @brief get all grounded predicates in KB, save in member variable
         * @return this function performs a call to getAllGroundedPredicates(), it returns what it receives from it
         */
        bool saveAllGroundedPredicates();
        
        /**
         * @brief print all predicates in internal KB, SaveAllGroundedPredicates() needs to be called first
         * @return false if internal_kb_ is empty, true otherwise
         */
        bool printInternalKB();
        
        /**
         * @brief save internal KB in second member variable for reset purposes: in case user wants to go
         * back to previoud KB version, without having to perform another service call
         */
        void backupInternalKB();
        
        /**
         * @brief find (grounded) predicate inside KB, return by reference an iterator to the element
         * @param predicate_name the name of the predicate to search in the KB
         * @param args the predicate grounded parameters
         * @param kiit return value gets written here by reference
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(std::string &predicate_name, std::vector<std::string> &args,
                std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit);
        
        /**
         * @brief overloaded function offered to call FindFactInternal() when we don't care about the returned iterator
         * @param predicate_name the name of the predicate to search in the KB
         * @param args the predicate grounded parameters
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(std::string &predicate_name, std::vector<std::string> &args);
        
        /**
         * @brief overloaded function offered to call FindFactInternal() when we don't care about the returned iterator
         * and args are empty
         * @param predicate_name the name of the predicate to search in the KB
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(std::string &predicate_name);
        
        /**
         * @brief overloaded function that alows to find predicate in KB with an input DomainFormula
         * (which can store a predicate)
         * @param predicate the fact that will be searched in KB
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(rosplan_knowledge_msgs::DomainFormula &predicate);
        
        /**
         * @brief remove fact from internal KB
         * @param predicate_name the name of the predicate to be removed from KB
         * @param args the grounded predicate parameters
         * @return true if predicate was found, false otherwise
         */
        bool removeFactInternal(std::string &predicate_name, std::vector<std::string> &args);
        
        /**
         * @brief remove fact from internal KB, this in an overloaded method that
         * accepts the predicate as a single list of strings
         * @param predicate_and_args single list of strings where the predicate can be expressed
         * @return true if predicate was found, false otherwise
         */
        bool removeFactInternal(std::vector<std::string> &predicate_and_args);
        
        /**
         * @brief check if action preconditions are consistent with internal KB information
         * @param action_name the name of the action to check if preconditions are met in KB
         * @return true if action is aplicable, false otherwise
         */
        bool isActionAplicable(std::string &action_name);
        
        /**
         * @brief apply delete and add list to KB current state
         * @param action_name the name of the action to be simulated
         * @return
         */
        bool simulateAction(std::string &action_name);

    private:
        // ros related variables
        ros::NodeHandle nh_;
        
        ros::ServiceClient od_srv_client_, on_srv_client_, dp_srv_client_, sp_srv_client_;
        
        // store a list of all domain operator names
        std::vector<std::string> operator_names_;
        
        // store a list of ungrounded domain operator details
        std::vector<rosplan_knowledge_msgs::DomainOperator> domain_operator_details_;
        
        // store a map of operator_names_ vs domain_operator_details_
        std::map<std::string, rosplan_knowledge_msgs::DomainOperator> domain_operator_map_;

        // store a list of ungrounded domain predicate details
        std::vector<rosplan_knowledge_msgs::DomainFormula> domain_predicate_details_;
        
        // store a list of domain predicates
        std::vector<std::string> domain_predicates_;
        
        // stores all grounded facts (is like an internal copy of KB)
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> internal_kb_;
        
        // stores all grounded facts (bkp of internal_kb_)
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> internal_kb_bkp_;
};
