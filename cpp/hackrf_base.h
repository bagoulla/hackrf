#ifndef HACKRF_BASE_IMPL_BASE_H
#define HACKRF_BASE_IMPL_BASE_H

#include <boost/thread.hpp>
#include <frontend/frontend.h>
#include <ossie/ThreadedComponent.h>

#include <frontend/frontend.h>
#include <bulkio/bulkio.h>
#include "struct_props.h"

#define BOOL_VALUE_HERE 0

class hackrf_base : public frontend::FrontendTunerDevice<frontend_tuner_status_struct_struct>, public virtual frontend::digital_tuner_delegation, public virtual frontend::rfinfo_delegation, protected ThreadedComponent
{
    public:
        hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        hackrf_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~hackrf_base();

        void start() throw (CF::Resource::StartError, CORBA::SystemException);

        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        void loadProperties();
        void removeAllocationIdRouting(const size_t tuner_id);

        virtual CF::Properties* getTunerStatus(const std::string& allocation_id);
        virtual void assignListener(const std::string& listen_alloc_id, const std::string& allocation_id);
        virtual void removeListener(const std::string& listen_alloc_id);
        void frontendTunerStatusChanged(const std::vector<frontend_tuner_status_struct_struct>* oldValue, const std::vector<frontend_tuner_status_struct_struct>* newValue);

    protected:
        // Member variables exposed as properties
        /// Property: serial_number
        std::string serial_number;
        /// Property: gain_settings
        gain_settings_struct gain_settings;

        // Ports
        /// Port: RFInfo_in
        frontend::InRFInfoPort *RFInfo_in;
        /// Port: DigitalTuner_in
        frontend::InDigitalTunerPort *DigitalTuner_in;
        /// Port: dataOctet_out
        bulkio::OutOctetPort *dataOctet_out;

        std::map<std::string, std::string> listeners;

        virtual void setNumChannels(size_t num);
        virtual void setNumChannels(size_t num, std::string tuner_type);

    private:
        void construct();
};
#endif // HACKRF_BASE_IMPL_BASE_H
