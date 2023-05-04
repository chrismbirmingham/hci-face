import Head from 'next/head';
import Chatbot from '../public/Chatbot.svg'
import TalkingRobot from '../public/TalkingRobot.svg'
import Puppetshow from '../public/Puppetshow.svg'
import Recording from '../public/Recording-pana.svg'
import Deadline from '../public/Deadline-pana.svg'
import Image from 'next/image';
import Mic from '../public/Mic.svg'
import Face from '../public/Face.svg'
import Listen from '../public/Listen.svg'

export default function Home() {
  const grid_box_theme = "max-w-sm p-6 bg-sky-500 border border-gray-200 rounded-lg shadow dark:bg-gray-800 dark:border-gray-700"
  const bold_subtitle = "mb-2 text-2xl font-bold tracking-tight text-gray-900 dark:text-white"

  return (
    <div className='Front Page'>
      <Head>
        <title>HCI FACE</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>

      <main className='text-2xl p-2'>
        <div className='text-4xl p-8'>
          Welcome to HCI Face!
        </div>

        <p className='text-2xl p-8'>The Human-Computer Interaction Facial Animation and Conversation Engine is a tool for developing new audio and text based interactions with new conversational AI.</p>
        <p className='text-2xl'>This open source tool makes it easy for developers to create interactive experiences with digital personas. 
                                The website is a work in progress, but provides a few examples of what can be done with the tool.
                                The backend can be configured for entirely offline use with free and open source models and software or can utilized online with paid cloud services.
                                The HCI-FACE backend provides the key components for an interactive experience:</p>
        <ul className='text-2xl p-2 list-disc list-inside'>
          <li>Text to Speech</li>
          <li>Speech to Text</li>
          <li>Facial Animation (Emotions and Visemes)</li>
          <li>LLM Based Conversation</li>
        </ul>
        <p className='text-2xl'>Together, these elements have been combined to create the following applets:</p>
        <div id="fullWidthTabContent" className="resize">
          <div className="max-w-fit p-4 bg-white rounded-lg md:p-8 dark:bg-gray-800" id="stats" role="tabpanel" aria-labelledby="stats-tab">
              <dl className="grid max-w-fit grid-cols-2 gap-8 p-4 mx-auto text-gray-900 sm:grid-cols-1 md:grid-cols-2 xl:grid-cols-4 dark:text-white sm:p-8">
                  
                <div className={grid_box_theme}>
                  <a href="/multimodal_interaction">
                    <Image priority src={TalkingRobot} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>Have a conversation</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Have a fully interactive conversation with an AI. Set the prompt and start a conversation!</p>
                </div>
                <div className={grid_box_theme}>
                  <a href="/transcription">
                    <Image priority src={Recording} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>Record your ideas</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Have your browser write down your thoughts. You can use buttons to control a microphone or have it intelligently take notes on your next idea session.</p>
                </div>
                <div className={grid_box_theme}>
                  <a href="/text_conversation">
                    <Image priority src={Chatbot} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>Text-chat with AI</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Have a text-based conversation with a Large Language Model. There are preset prompts or you can create your own!</p>
                </div>
                <div className={grid_box_theme}>
                  <a href="/puppeteering">
                    <Image priority src={Puppetshow} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>Puppeteer the face</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Visualize your robot face's behaviors and emotional expressions. You can also see and hear speech the face as it says what you want it to.</p>
                </div>
                <div className={grid_box_theme}>
                  <a href="/clock">
                    <Image priority src={Deadline} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>Work with the clock</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Timer, Alarm Clock, Stopwatch, and Pomodoro Focus Timer all in one!</p>
                </div>
                <div className={grid_box_theme}>
                  <a href="/mic_playground">
                    <Image priority src={Mic} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>Setup/Test your mic</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Visualize the sound waves, record with button controls or have the microphone pick out your voice as you talk automatically.</p>
                </div>
                <div className={grid_box_theme}>
                  <a href="/playfaces">
                    <Image priority src={Face} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>Visualize new expressions</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Directly control the facial action units to customize the face's expression. See how it appears on a range of possible faces.</p>
                </div>
                <div className={grid_box_theme}>
                  <a href="/reader">
                    <Image priority src={Listen} alt='next.js logo'/>
                      <h5 className={bold_subtitle}>List to your text read aloud</h5>
                  </a>
                  <p className="mb-3 font-normal text-gray-700 dark:text-gray-400">Hear a voice read your text at your preferred speed.</p>
                </div>
              </dl>
      <a href="https://storyset.com/happy">Happy illustrations by Storyset</a>
          </div>
        </div>
        <h3 className="text-3xl">
            Additional Pages:
        </h3>
        <br></br>Show just the face on <a className="text-blue-600" href="/face">this page!</a>
      </main>
    </div>
  )
}
