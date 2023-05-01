import '@styles/globals.css'
import Layout from '@components/layout/pagelayout';


export default function App({ Component, pageProps, router }) {
  if (router.pathname === '/face') {
    return <Component {...pageProps} />
  }
  return (
    <Layout>
      <Component {...pageProps} />
    </Layout>
  )
}
