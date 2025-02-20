const { app } = require('@azure/functions');

app.eventGrid('eventGridTrigger2', {
    handler: (event, context) => {
        context.log('Event grid function processed event:', event);
    }
});
