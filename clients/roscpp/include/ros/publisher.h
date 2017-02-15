/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_PUBLISHER_HANDLE_H
#define ROSCPP_PUBLISHER_HANDLE_H

#include <type_traits>
#include "ros/forwards.h"
#include "ros/common.h"
#include "ros/message.h"
#include "ros/serialization.h"
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include "ros/master.h"
#include "ros/this_node.h"
#include "std_msgs/DataAssociation.h"
#include "std_msgs/NamedReference.h"

namespace ros
{
  typedef std::pair<std::string, std::string> StringPair;
  typedef std::list<StringPair> AssociationList;

  /**
   * \brief Manages an advertisement on a specific topic.
   *
   * A Publisher should always be created through a call to NodeHandle::advertise(), or copied from one
   * that was. Once all copies of a specific
   * Publisher go out of scope, any subscriber status callbacks associated with that handle will stop
   * being called.  Once all Publishers for a given topic go out of scope the topic will be unadvertised.
   */
  class ROSCPP_DECL Publisher
  {
  public:
    Publisher() {}
    Publisher(const Publisher& rhs);
    ~Publisher();

    template <typename M>
    void publish(const boost::shared_ptr<M>& message)
    {
        static ros::Time last_called = ros::Time(0.0);
        if((ros::Time::now() - last_called).toSec() > 10.0)
        {
            ROS_WARN_STREAM("Publisher::publish is deprecated and introduces an additional message copy operation, please consider using Publisher::publish_get_guid.");
            last_called = ros::Time::now();
        }
        typename std::remove_const<M>::type message_ = *message;
        publish_get_guid(message_);
    }

    template <typename M>
    void publish(const M& message)
    {
        static ros::Time last_called = ros::Time(0.0);
        if((ros::Time::now() - last_called).toSec() > 10.0)
        {
            ROS_WARN_STREAM("Publisher::publish is deprecated and introduces an additional message copy operation, please consider using Publisher::publish_get_guid.");
            last_called = ros::Time::now();
        }
        M message_ = message;
        publish_get_guid(message_);
    }

    template <typename M>
    void publish_get_guid(boost::shared_ptr<M>& message)
    {
        message->guid = nextGUID();
        associated_publish_impl(*message, ros::AssociationList());
    }

    template <typename M>
    void associated_publish(boost::shared_ptr<M>& message, const std::list<std::string>& associated_ids)
    {
      ros::AssociationList assoc_ids = ros::AssociationList();
      for (std::list<std::string>::const_iterator iterator = associated_ids.begin(), end = associated_ids.end(); iterator != end; ++iterator)
      {
        assoc_ids.push_back(std::pair<std::string, std::string>("None", *iterator));
      }

      message->guid = nextGUID();
      associated_publish_impl(message, assoc_ids);
    }

    template <typename M>
    void associated_publish(boost::shared_ptr<M>& message, const ros::AssociationList& associated_ids)
    {
        message->guid = nextGUID();
        associated_publish_impl(message, associated_ids);
    }

    template <typename M>
    void publish_get_guid(M& message)
    {
        message.guid = nextGUID();
        associated_publish_impl(message, ros::AssociationList());
    }

    template <typename M>
    void associated_publish(M& message, const std::list<std::string>& associated_ids)
    {
        ros::AssociationList assoc_ids = ros::AssociationList();
        for (std::list<std::string>::const_iterator iterator = associated_ids.begin(), end = associated_ids.end(); iterator != end; ++iterator)
        {
            assoc_ids.push_back(std::pair<std::string, std::string>("None", *iterator));
        }

        message.guid = nextGUID();
        associated_publish_impl(message, assoc_ids);
    }

    template <typename M>
    void associated_publish(M& message, const ros::AssociationList& associated_ids)
    {
        message.guid = nextGUID();
        associated_publish_impl(message, associated_ids);
    }

    /**
     * \brief Shutdown the advertisement associated with this Publisher
     *
     * This method usually does not need to be explicitly called, as automatic shutdown happens when
     * all copies of this Publisher go out of scope
     *
     * This method overrides the automatic reference counted unadvertise, and does so immediately.
     * \note Note that if multiple advertisements were made through NodeHandle::advertise(), this will
     * only remove the one associated with this Publisher
     */
    void shutdown();

    /**
     * \brief Returns the topic that this Publisher will publish on.
     */
    std::string getTopic() const;

    /**
     * \brief Returns the number of subscribers that are currently connected to this Publisher
     */
    uint32_t getNumSubscribers() const;

    /**
     * \brief Returns whether or not this topic is latched
     */
    bool isLatched() const;

    operator void*() const { return (impl_ && impl_->isValid()) ? (void*)1 : (void*)0; }

    bool operator<(const Publisher& rhs) const
    {
      return impl_ < rhs.impl_;
    }

    bool operator==(const Publisher& rhs) const
    {
      return impl_ == rhs.impl_;
    }

    bool operator!=(const Publisher& rhs) const
    {
      return impl_ != rhs.impl_;
    }

  private:

    Publisher(const std::string& topic, const std::string& md5sum, 
              const std::string& datatype, const NodeHandle& node_handle, 
              const SubscriberCallbacksPtr& callbacks);

    template <typename M>
    void associated_publish_impl(const boost::shared_ptr<M>& message, const ros::AssociationList& associated_ids) const
    {
      using namespace serialization;

      if (!impl_)
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher");
          return;
        }

      if (!impl_->isValid())
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
          return;
        }

      ROS_ASSERT_MSG(impl_->md5sum_ == "*" || std::string(mt::md5sum<M>(*message)) == "*" || impl_->md5sum_ == mt::md5sum<M>(*message),
                     "Trying to publish message of type [%s/%s] on a publisher with type [%s/%s]",
                     mt::datatype<M>(*message), mt::md5sum<M>(*message),
                     impl_->datatype_.c_str(), impl_->md5sum_.c_str());

      SerializedMessage m;
      m.type_info = &typeid(M);
      m.message = message;

      publish(boost::bind(serializeMessage<M>, boost::ref(message)), m);

      if(!associated_ids.empty())
      {
        std_msgs::DataAssociation da_msg;
        da_msg.parent_id = message->guid;
        da_msg.header.stamp = ros::Time::now();
        for(ros::AssociationList::const_iterator iterator = associated_ids.begin(); iterator != associated_ids.end(); ++iterator)
        {
          std_msgs::NamedReference ref;
          ref.reference_name = iterator->first;
          ref.reference_id = iterator->second;
          da_msg.associated_ids.push_back(ref);
        }
        publishDataAssociation(da_msg);
      }
    }

    template <typename M>
    void associated_publish_impl(const M& message, const ros::AssociationList& associated_ids) const
    {
      using namespace serialization;
      namespace mt = ros::message_traits;

      if (!impl_)
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher");
          return;
        }

      if (!impl_->isValid())
        {
          ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher (topic [%s])", impl_->topic_.c_str());
          return;
        }

      ROS_ASSERT_MSG(impl_->md5sum_ == "*" || std::string(mt::md5sum<M>(message)) == "*" || impl_->md5sum_ == mt::md5sum<M>(message),
                     "Trying to publish message of type [%s/%s] on a publisher with type [%s/%s]",
                     mt::datatype<M>(message), mt::md5sum<M>(message),
                     impl_->datatype_.c_str(), impl_->md5sum_.c_str());

      SerializedMessage m;
      publish(boost::bind(serializeMessage<M>, boost::ref(message)), m);

      if(!associated_ids.empty())
      {
        std_msgs::DataAssociation da_msg;
        da_msg.parent_id = message.guid;
        da_msg.header.stamp = ros::Time::now();
        for(ros::AssociationList::const_iterator iterator = associated_ids.begin(); iterator != associated_ids.end(); ++iterator)
        {
          std_msgs::NamedReference ref;
          ref.reference_name = iterator->first;
          ref.reference_id = iterator->second;
          da_msg.associated_ids.push_back(ref);
        }
        publishDataAssociation(da_msg);
      }
    }

    void publish(const boost::function<SerializedMessage(void)>& serfunc, SerializedMessage& m) const;
    void incrementSequence() const;
    void publishDataAssociation(std_msgs::DataAssociation& msg) const;
    std::string nextGUID();

    class ROSCPP_DECL Impl
    {
    public:
      Impl();
      ~Impl();

      void unadvertise();
      bool isValid() const;

      std::string topic_;
      std::string md5sum_;
      std::string datatype_;
      NodeHandlePtr node_handle_;
      SubscriberCallbacksPtr callbacks_;
      bool unadvertised_;

      std::string guid_prefix;
      uint32_t seq_;
      boost::mutex seq_mutex_;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    typedef boost::weak_ptr<Impl> ImplWPtr;

    ImplPtr impl_;

    friend class NodeHandle;
    friend class NodeHandleBackingCollection;
  };

  typedef std::vector<Publisher> V_Publisher;
}

#endif // ROSCPP_PUBLISHER_HANDLE_H

