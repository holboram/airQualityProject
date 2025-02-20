const { app } = require('@azure/functions');

app.eventGrid('eventGridTrigger1', {
    handler: (event, context) => {
        const newSiteName = event.data && event.data.newSiteName ? event.data.newSiteName : 'defaultSiteName';
        if (newSiteName === null || newSiteName === undefined) {
            context.log('Error: newSiteName is null or undefined');
            return;
        }
        context.log('Event grid function processed event:', event);
        context.log('New site name:', newSiteName);
    }
});